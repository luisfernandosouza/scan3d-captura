"""
Passo 3: extrai a arquitetura do ambiente a partir da nuvem de pontos densa.

Detecta piso, teto, paredes e aberturas (portas e janelas) só por geometria,
medindo o que foi capturado, e gera:
    resultados/<captura>/planta.svg      planta baixa com cotas
    resultados/<captura>/planta.dxf      a mesma planta para AutoCAD/LibreCAD
    resultados/<captura>/ambiente.json   medidas de piso, teto, paredes e aberturas

Uso:
    py -3 planta.py quarto
    py -3 planta.py quarto --escala 0.397   (multiplica as medidas, se precisar)

Convenções: metros, Y para cima (como o modelo exportado).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


def log(msg: str) -> None:
    print(msg, flush=True)


# ------------------------------------------------------------------ leitura

def load_cloud(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    from plyfile import PlyData
    v = PlyData.read(str(path))["vertex"]
    xyz = np.column_stack([v["x"], v["y"], v["z"]]).astype(np.float64)
    names = v.data.dtype.names
    normals = None
    if all(n in names for n in ("nx", "ny", "nz")):
        normals = np.column_stack([v["nx"], v["ny"], v["nz"]]).astype(np.float64)
        length = np.linalg.norm(normals, axis=1)
        ok = length > 1e-6
        normals[ok] /= length[ok, None]
    return xyz, normals


def voxel_downsample(xyz: np.ndarray, normals: np.ndarray | None, size: float):
    keys = np.floor(xyz / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[idx], (normals[idx] if normals is not None else None)


def estimate_normals(xyz: np.ndarray, k: int = 12) -> np.ndarray:
    """Normais por PCA local (só se a nuvem não trouxer normais)."""
    from scipy.spatial import cKDTree
    tree = cKDTree(xyz)
    _, nn = tree.query(xyz, k=k)
    pts = xyz[nn] - xyz[nn].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", pts, pts)
    _, vecs = np.linalg.eigh(cov)
    return vecs[:, :, 0]


# ------------------------------------------------------------- detecção

def histogram_peaks(values: np.ndarray, bin_size: float, min_count: int,
                    min_separation: float) -> list[tuple[float, int]]:
    """Picos de um histograma 1D: (posição, quantidade de pontos)."""
    if len(values) == 0:
        return []
    lo, hi = values.min(), values.max()
    bins = max(1, int(math.ceil((hi - lo) / bin_size)))
    counts, edges = np.histogram(values, bins=bins, range=(lo, lo + bins * bin_size))
    # Suaviza levemente para não dividir um plano em dois bins.
    smooth = np.convolve(counts, [1, 2, 1], mode="same") / 4
    order = np.argsort(smooth)[::-1]
    peaks: list[tuple[float, int]] = []
    for i in order:
        if smooth[i] < min_count:
            break
        pos = (edges[i] + edges[i + 1]) / 2
        if all(abs(pos - p) >= min_separation for p, _ in peaks):
            lo_i, hi_i = max(0, i - 1), min(len(counts), i + 2)
            peaks.append((float(pos), int(counts[lo_i:hi_i].sum())))
    return peaks


def find_floor_ceiling(xyz: np.ndarray, normals: np.ndarray) -> tuple[float, float]:
    horiz = np.abs(normals[:, 1]) > 0.85
    y = xyz[horiz, 1]
    if len(y) < 100:
        y = xyz[:, 1]
    peaks = histogram_peaks(y, 0.02, max(50, len(y) * 0.01), 0.5)
    if not peaks:
        return float(np.percentile(xyz[:, 1], 1)), float(np.percentile(xyz[:, 1], 99))
    ys = sorted(p for p, _ in peaks)
    mid = np.median(xyz[:, 1])
    below = [p for p, c in peaks if p < mid]
    above = [p for p, c in peaks if p > mid]
    # Piso: o plano horizontal mais forte embaixo; teto: o mais forte em cima.
    floor = max(((p, c) for p, c in peaks if p < mid), key=lambda pc: pc[1], default=(ys[0], 0))[0]
    ceiling = max(((p, c) for p, c in peaks if p > mid), key=lambda pc: pc[1], default=(ys[-1], 0))[0]
    if not below:
        floor = float(np.percentile(xyz[:, 1], 1))
    if not above:
        ceiling = float(np.percentile(xyz[:, 1], 99))
    return _refine(y, floor), _refine(y, ceiling)


def _refine(values: np.ndarray, pos: float, radius: float = 0.03) -> float:
    """Troca o centro do bin pela mediana dos pontos perto do pico (precisão < 1 cm)."""
    near = values[np.abs(values - pos) < radius]
    return float(np.median(near)) if len(near) else float(pos)


def dominant_angle(normals_xz: np.ndarray) -> float:
    """Direção principal das paredes (supõe paredes em ângulo reto entre si)."""
    ang = np.arctan2(normals_xz[:, 1], normals_xz[:, 0])
    a4 = np.mod(ang * 4, 2 * np.pi)  # dobra 0/90/180/270 graus no mesmo ponto
    c, s = np.cos(a4).sum(), np.sin(a4).sum()
    return float(np.arctan2(s, c) / 4)


def find_walls(xyz: np.ndarray, normals: np.ndarray, floor: float, ceiling: float):
    height = ceiling - floor
    band = (xyz[:, 1] > floor + 0.15 * height) & (xyz[:, 1] < ceiling - 0.1 * height)
    vert = np.abs(normals[:, 1]) < 0.3
    sel = band & vert
    pts = xyz[sel]
    nrm = normals[sel][:, [0, 2]]
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9)
    theta = dominant_angle(nrm)
    c, s = math.cos(-theta), math.sin(-theta)
    rot = np.array([[c, -s], [s, c]])  # gira o plano XZ para alinhar paredes aos eixos
    uv = pts[:, [0, 2]] @ rot.T
    n_uv = nrm @ rot.T

    walls = []
    for axis in (0, 1):  # 0: paredes perpendiculares a U; 1: perpendiculares a V
        on_axis = np.abs(n_uv[:, axis]) > 0.9
        coords = uv[on_axis, axis]
        other = uv[on_axis, 1 - axis]
        min_pts = max(80, int(on_axis.sum() * 0.03))
        for pos, count in histogram_peaks(coords, 0.03, min_pts, 0.25):
            pos = _refine(coords, pos)
            near = np.abs(coords - pos) < 0.06
            span = other[near]
            if len(span) < min_pts:
                continue
            a, b = np.percentile(span, 1), np.percentile(span, 99)
            if b - a < 0.4:
                continue
            walls.append({"axis": axis, "pos": float(pos), "start": float(a), "end": float(b),
                          "points": int(near.sum())})
    return walls, theta, rot


def keep_room_walls(walls: list[dict], uv_all: np.ndarray) -> list[dict]:
    """Mantém, em cada eixo, as paredes do contorno (as mais externas com muitos pontos)."""
    result = []
    for axis in (0, 1):
        group = sorted([w for w in walls if w["axis"] == axis], key=lambda w: w["pos"])
        if not group:
            continue
        strong = [w for w in group if w["points"] >= 0.25 * max(g["points"] for g in group)]
        result.extend(strong)
    snap_corners(result)
    return result


def snap_corners(walls: list[dict], tol: float = 0.3) -> None:
    """Leva a ponta de cada parede até a parede perpendicular mais próxima (fecha os cantos)."""
    for w in walls:
        perp = [o["pos"] for o in walls if o["axis"] != w["axis"]
                and o["start"] - tol <= w["pos"] <= o["end"] + tol]
        for key in ("start", "end"):
            best = min(perp, key=lambda p: abs(p - w[key]), default=None)
            if best is not None and abs(best - w[key]) < tol:
                w[key] = float(best)


def find_openings(xyz: np.ndarray, rot: np.ndarray, wall: dict, floor: float, ceiling: float,
                  cell: float = 0.05) -> list[dict]:
    """
    Procura buracos retangulares na parede: regiões sem pontos dentro do plano.
    Porta: começa no piso. Janela: começa acima do piso.
    """
    uv = xyz[:, [0, 2]] @ rot.T
    axis = wall["axis"]
    near = np.abs(uv[:, axis] - wall["pos"]) < 0.08
    s = uv[near, 1 - axis]
    h = xyz[near, 1] - floor
    height = ceiling - floor
    ns = int(math.ceil((wall["end"] - wall["start"]) / cell))
    nh = int(math.ceil(height / cell))
    if ns < 4 or nh < 4:
        return []
    grid = np.zeros((nh, ns), bool)
    si = ((s - wall["start"]) / cell).astype(int)
    hi = (h / cell).astype(int)
    ok = (si >= 0) & (si < ns) & (hi >= 0) & (hi < nh)
    grid[hi[ok], si[ok]] = True
    # Fecha pequenos furos de ruído (1 célula).
    filled = grid.copy()
    filled[1:-1, 1:-1] |= (grid[:-2, 1:-1] & grid[2:, 1:-1]) | (grid[1:-1, :-2] & grid[1:-1, 2:])

    empty = ~filled
    from collections import deque
    seen = np.zeros_like(empty)
    openings = []
    for r in range(nh):
        for c in range(ns):
            if not empty[r, c] or seen[r, c]:
                continue
            q = deque([(r, c)])
            seen[r, c] = True
            cells = []
            while q:
                y, x = q.popleft()
                cells.append((y, x))
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < nh and 0 <= xx < ns and empty[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        q.append((yy, xx))
            ys = [y for y, _ in cells]
            xs = [x for _, x in cells]
            r0, r1, c0, c1 = min(ys), max(ys) + 1, min(xs), max(xs) + 1
            w, hh = (c1 - c0) * cell, (r1 - r0) * cell
            fill_ratio = len(cells) / ((r1 - r0) * (c1 - c0))
            if fill_ratio < 0.7 or w < 0.4 or hh < 0.4:
                continue
            if c0 == 0 or c1 == ns:
                continue  # encosta na ponta da parede: é fim de captura, não abertura
            bottom, top = r0 * cell, r1 * cell
            # Refina as bordas pelos pontos reais (a grade só tem 5 cm de precisão).
            mid_s = wall["start"] + (c0 + c1) / 2 * cell
            mid_h = (r0 + r1) / 2 * cell
            row_band = (h > bottom + 0.2 * hh) & (h < top - 0.2 * hh)
            col_band = np.abs(s - mid_s) < 0.3 * w
            left = s[row_band & (s < mid_s) & (s > mid_s - w)]
            right = s[row_band & (s > mid_s) & (s < mid_s + w)]
            s_a = float(left.max()) if len(left) else wall["start"] + c0 * cell
            s_b = float(right.min()) if len(right) else wall["start"] + c1 * cell
            w = s_b - s_a
            below = h[col_band & (h < mid_h) & (h > 0.1)]
            above = h[col_band & (h > mid_h)]
            if len(below) and bottom >= 0.15:
                bottom = float(below.max())
            elif bottom < 0.15:
                bottom = 0.0  # porta: vai até o piso (o que aparece embaixo é o próprio piso)
            if len(above):
                top = float(above.min())
            hh = top - bottom
            c0_m = s_a
            if bottom < 0.15 and 1.7 <= hh <= 2.5 and 0.6 <= w <= 1.6:
                kind = "porta"
            elif bottom >= 0.3 and top <= height - 0.05 and w <= 3.0:
                kind = "janela"
            else:
                continue
            openings.append({
                "tipo": kind,
                "inicio_m": round(c0_m, 3),
                "largura_m": round(w, 2),
                "altura_m": round(hh, 2),
                "peitoril_m": round(bottom, 2),
                "confianca": "media" if fill_ratio > 0.85 else "baixa",
            })
    return openings


# ------------------------------------------------------------------ saída

def to_xz(rot: np.ndarray, u: float, v: float) -> tuple[float, float]:
    x, z = np.array([u, v]) @ rot  # inversa da rotação (rot é ortogonal)
    return float(x), float(z)


def wall_endpoints(w: dict) -> tuple[tuple[float, float], tuple[float, float]]:
    if w["axis"] == 0:
        return (w["pos"], w["start"]), (w["pos"], w["end"])
    return (w["start"], w["pos"]), (w["end"], w["pos"])


def write_svg(path: Path, walls: list[dict], scale_px: float = 100.0) -> None:
    pts = [p for w in walls for p in wall_endpoints(w)]
    us, vs = [p[0] for p in pts], [p[1] for p in pts]
    pad = 0.8
    u0, v0 = min(us) - pad, min(vs) - pad
    W = (max(us) - min(us) + 2 * pad) * scale_px
    H = (max(vs) - min(vs) + 2 * pad) * scale_px

    def P(u, v):
        return (u - u0) * scale_px, (v - v0) * scale_px

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" '
           f'viewBox="0 0 {W:.0f} {H:.0f}" font-family="Arial" font-size="14">',
           f'<rect width="100%" height="100%" fill="white"/>']
    for w in walls:
        (a, b) = wall_endpoints(w)
        x1, y1 = P(*a)
        x2, y2 = P(*b)
        out.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                   f'stroke="black" stroke-width="8" stroke-linecap="square"/>')
        length = w["end"] - w["start"]
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        off = 22
        if w["axis"] == 0:
            out.append(f'<text x="{mx + off:.1f}" y="{my:.1f}" fill="#1a56db" '
                       f'transform="rotate(90 {mx + off:.1f} {my:.1f})" text-anchor="middle">'
                       f'{length:.2f} m</text>')
        else:
            out.append(f'<text x="{mx:.1f}" y="{my - off + 8:.1f}" fill="#1a56db" '
                       f'text-anchor="middle">{length:.2f} m</text>')
        for o in w.get("aberturas", []):
            s0 = o["inicio_m"]
            s1 = s0 + o["largura_m"]
            if w["axis"] == 0:
                a2, b2 = P(w["pos"], s0), P(w["pos"], s1)
            else:
                a2, b2 = P(s0, w["pos"]), P(s1, w["pos"])
            color = "#d97706" if o["tipo"] == "porta" else "#0891b2"
            out.append(f'<line x1="{a2[0]:.1f}" y1="{a2[1]:.1f}" x2="{b2[0]:.1f}" y2="{b2[1]:.1f}" '
                       f'stroke="{color}" stroke-width="10"/>')
            tx, ty = (a2[0] + b2[0]) / 2, (a2[1] + b2[1]) / 2
            out.append(f'<text x="{tx:.1f}" y="{ty + 30:.1f}" fill="{color}" text-anchor="middle" '
                       f'font-size="12">{o["tipo"]} {o["largura_m"]:.2f}×{o["altura_m"]:.2f}</text>')
    out.append(f'<text x="10" y="{H - 10:.0f}" fill="#555" font-size="12">'
               f'Planta gerada pelo Scan3D: medidas em metros, pela nuvem de pontos capturada</text>')
    out.append("</svg>")
    path.write_text("\n".join(out), encoding="utf-8")


def write_dxf(path: Path, walls: list[dict]) -> None:
    """DXF mínimo (R12): linhas das paredes em PAREDES, aberturas em ABERTURAS e cotas em COTAS."""
    ents = []

    def line(layer, a, b):
        ents.append(f"0\nLINE\n8\n{layer}\n10\n{a[0]:.4f}\n20\n{-a[1]:.4f}\n30\n0\n"
                    f"11\n{b[0]:.4f}\n21\n{-b[1]:.4f}\n31\n0")

    def text(layer, p, h, s, ang=0):
        ents.append(f"0\nTEXT\n8\n{layer}\n10\n{p[0]:.4f}\n20\n{-p[1]:.4f}\n30\n0\n"
                    f"40\n{h}\n1\n{s}\n50\n{ang}")

    for w in walls:
        a, b = wall_endpoints(w)
        line("PAREDES", a, b)
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        text("COTAS", (mid[0] + 0.1, mid[1]), 0.1, f"{w['end'] - w['start']:.2f}",
             90 if w["axis"] == 0 else 0)
        for o in w.get("aberturas", []):
            s0, s1 = o["inicio_m"], o["inicio_m"] + o["largura_m"]
            if w["axis"] == 0:
                line("ABERTURAS", (w["pos"], s0), (w["pos"], s1))
            else:
                line("ABERTURAS", (s0, w["pos"]), (s1, w["pos"]))
    body = "\n".join(ents)
    path.write_text(f"0\nSECTION\n2\nENTITIES\n{body}\n0\nENDSEC\n0\nEOF\n", encoding="ascii")


# ------------------------------------------------------------------- main

def resolve_scale(work: Path, result_dir: Path) -> float:
    """Se a IA de escala rodou depois do processamento, aplica o fator dela."""
    rel = result_dir / "relatorio.json"
    ia = work / "colmap" / "escala_ia.json"
    if not ia.is_file():
        return 1.0
    fonte = ""
    if rel.is_file():
        fonte = json.loads(rel.read_text(encoding="utf-8")).get("colmap", {}).get("escala_fonte", "")
    if fonte == "ia":
        return 1.0  # o modelo já saiu na escala da IA
    k = json.loads(ia.read_text(encoding="utf-8")).get("k")
    return float(k) if k else 1.0


def main() -> None:
    ap = argparse.ArgumentParser(description="Gera planta baixa e medidas do ambiente.")
    ap.add_argument("captura", help="nome da captura (ex.: quarto)")
    ap.add_argument("--escala", type=float, default=None,
                    help="fator para multiplicar as medidas (padrão: automático)")
    args = ap.parse_args()

    work = ROOT / "trabalho" / args.captura
    dest = ROOT / "resultados" / args.captura
    cloud = work / "denso" / "fused.ply"
    if not cloud.is_file():
        cloud = dest / "nuvem_densa.ply"
    if not cloud.is_file():
        raise SystemExit(f"Não achei a nuvem de pontos de '{args.captura}'. Rode o processar.bat antes.")

    scale = args.escala if args.escala else resolve_scale(work, dest)
    log(f"Lendo {cloud.name}" + (f" (escala x{scale:.3f})" if scale != 1.0 else ""))
    xyz, normals = load_cloud(cloud)
    xyz *= scale
    xyz, normals = voxel_downsample(xyz, normals, 0.02)
    if normals is None:
        normals = estimate_normals(xyz)
    log(f"  {len(xyz)} pontos após reduzir para 2 cm")

    floor, ceiling = find_floor_ceiling(xyz, normals)
    log(f"Piso em {floor:.2f} m, teto em {ceiling:.2f} m: pé-direito {ceiling - floor:.2f} m")

    walls, theta, rot = find_walls(xyz, normals, floor, ceiling)
    walls = keep_room_walls(walls, None)
    log(f"{len(walls)} paredes encontradas")
    for i, w in enumerate(walls, 1):
        w["aberturas"] = find_openings(xyz, rot, w, floor, ceiling)
        w["comprimento_m"] = round(w["end"] - w["start"], 2)
        log(f"  parede {i}: {w['comprimento_m']:.2f} m"
            + "".join(f" | {o['tipo']} {o['largura_m']:.2f}x{o['altura_m']:.2f} m" for o in w["aberturas"]))

    dest.mkdir(parents=True, exist_ok=True)
    data = {
        "unidade": "metros",
        "escala_aplicada": scale,
        "pe_direito_m": round(ceiling - floor, 2),
        "piso_y": round(floor, 3),
        "teto_y": round(ceiling, 3),
        "rotacao_planta_graus": round(math.degrees(theta), 2),
        "paredes": [{
            "comprimento_m": w["comprimento_m"],
            "inicio_xz": to_xz(rot, *wall_endpoints(w)[0]),
            "fim_xz": to_xz(rot, *wall_endpoints(w)[1]),
            "aberturas": w["aberturas"],
        } for w in walls],
        "observacao": "Medidas tiradas só dos pontos capturados. Aberturas com confiança baixa "
                      "podem ser áreas escondidas atrás de móveis.",
    }
    (dest / "ambiente.json").write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    write_svg(dest / "planta.svg", walls)
    write_dxf(dest / "planta.dxf", walls)
    log(f"Pronto: {dest / 'planta.svg'}, planta.dxf e ambiente.json")


if __name__ == "__main__":
    main()
