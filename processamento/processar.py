"""
Processa uma captura do app Scan3D Captura e gera um modelo 3D texturizado
em escala real, pronto para abrir no Blender.

Uso:
    py -3 processar.py CAPTURA.zip            (ou a pasta já descompactada)
    py -3 processar.py CAPTURA.zip --e57      (também gera nuvem de pontos .e57)

Etapas:
    1. Lê fotos, poses do ARCore e medidas de trena.
    2. COLMAP: extrai pontos de interesse, compara as fotos e triangula usando
       as poses do ARCore; refina e realinha ao sistema métrico do ARCore.
    3. OpenMVS: nuvem densa, malha e textura (sem fechar buracos).
    4. Copia o resultado para resultados/<captura>/.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sqlite3
import subprocess
import sys
import time
import zipfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
TOOLS = ROOT / "ferramentas"

# ARCore usa câmera estilo OpenGL (+Y cima, olhando para -Z).
# COLMAP usa câmera estilo OpenCV (+Y baixo, olhando para +Z).
GL_TO_CV = np.diag([1.0, -1.0, -1.0])


# ----------------------------------------------------------------- utilidades

def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def find_tool(name: str) -> str:
    """Procura um executável em ferramentas/ ou no PATH."""
    for candidate in (f"{name}.bat", f"{name}.exe", name):
        for hit in TOOLS.rglob(candidate) if TOOLS.exists() else []:
            if hit.is_file():
                return str(hit)
        found = shutil.which(candidate)
        if found:
            return found
    raise SystemExit(
        f"Não encontrei '{name}'. Rode instalar.bat primeiro."
    )


def run(cmd: list, cwd: Path | None = None, allow_fail: bool = False) -> bool:
    log("$ " + " ".join(str(c) for c in cmd))
    result = subprocess.run([str(c) for c in cmd], cwd=cwd)
    if result.returncode != 0:
        if allow_fail:
            log(f"  (falhou com código {result.returncode}, seguindo sem esta etapa)")
            return False
        raise SystemExit(f"Etapa falhou (código {result.returncode}): {cmd[0]}")
    return True


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat_wxyz(r: np.ndarray) -> tuple[float, float, float, float]:
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return w, x, y, z


def arcore_to_colmap(t: list, q_xyzw: list, scale: float):
    """Pose do ARCore (câmera->mundo, OpenGL) para COLMAP (mundo->câmera, OpenCV)."""
    r_c2w = quat_to_rot(*q_xyzw) @ GL_TO_CV
    center = np.asarray(t, dtype=float) * scale
    r_w2c = r_c2w.T
    tvec = -r_w2c @ center
    return rot_to_quat_wxyz(r_w2c), tvec, center


# ------------------------------------------------------------ leitura da captura

def open_capture(src: Path, work: Path) -> Path:
    """Aceita o .zip exportado pelo app ou a pasta da captura. Devolve a pasta."""
    if src.is_dir():
        return src
    if src.suffix.lower() != ".zip":
        raise SystemExit("Informe o .zip exportado pelo app ou a pasta da captura.")
    dest = work / "captura"
    if not dest.exists():
        log(f"Descompactando {src.name}")
        with zipfile.ZipFile(src) as z:
            z.extractall(dest)
    sessions = [p.parent for p in dest.rglob("frames.jsonl")]
    if not sessions:
        raise SystemExit("O .zip não tem frames.jsonl. É uma captura do Scan3D?")
    return sessions[0]


def load_frames(cap: Path) -> list[dict]:
    frames = []
    with open(cap / "frames.jsonl", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            fr = json.loads(line)
            if (cap / fr["image"]).is_file():
                frames.append(fr)
    frames.sort(key=lambda fr: fr["index"])
    return frames


def scale_from_calibration(cap: Path) -> tuple[float, list[dict]]:
    """
    Correção de escala pelas medidas de trena (mediana de real/ARCore).
    Medidas absurdas (ARCore perdido) são descartadas.
    """
    path = cap / "calibration.json"
    if not path.is_file():
        return 1.0, []
    data = json.loads(path.read_text(encoding="utf-8"))
    report, ratios = [], []
    for m in data.get("measurements", []):
        a, r = m.get("arcore_m", 0), m.get("real_m", 0)
        if a <= 0.05 or r <= 0:
            continue
        ok = 0.7 <= r / a <= 1.4
        if ok:
            ratios.append(r / a)
        report.append({"real_cm": round(r * 100, 1), "arcore_cm": round(a * 100, 1),
                       "usada": ok})
    if not ratios:
        return 1.0, report
    return float(np.median(ratios)), report


# ------------------------------------------------------- modelo COLMAP em texto

def read_txt_model(d: Path):
    images = {}
    lines = [l for l in (d / "images.txt").read_text().splitlines() if not l.startswith("#")]
    i = 0
    while i < len(lines):
        if not lines[i].strip():
            i += 1
            continue
        parts = lines[i].split()
        iid = int(parts[0])
        q = [float(x) for x in parts[1:5]]
        t = np.array([float(x) for x in parts[5:8]])
        images[iid] = {"q": q, "t": t, "cam": parts[8], "name": " ".join(parts[9:]),
                       "pts": lines[i + 1] if i + 1 < len(lines) else ""}
        i += 2
    points = []
    for l in (d / "points3D.txt").read_text().splitlines():
        if l.startswith("#") or not l.strip():
            continue
        parts = l.split()
        points.append((parts[0], np.array([float(x) for x in parts[1:4]]), parts[4:]))
    return images, points


def camera_center(img: dict) -> np.ndarray:
    qw, qx, qy, qz = img["q"]
    r = quat_to_rot(qx, qy, qz, qw)
    return -r.T @ img["t"]


def write_transformed_model(src: Path, dst: Path, scale: float, rot: np.ndarray,
                            offset: np.ndarray) -> None:
    """Aplica X' = scale * rot @ X + offset a câmeras e pontos."""
    images, points = read_txt_model(src)
    dst.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src / "cameras.txt", dst / "cameras.txt")
    with open(dst / "images.txt", "w") as f:
        for iid, img in images.items():
            qw, qx, qy, qz = img["q"]
            r_w2c = quat_to_rot(qx, qy, qz, qw) @ rot.T
            t = scale * img["t"] - r_w2c @ offset
            w, x, y, z = rot_to_quat_wxyz(r_w2c)
            f.write(f"{iid} {w} {x} {y} {z} {t[0]} {t[1]} {t[2]} {img['cam']} {img['name']}\n{img['pts']}\n")
    with open(dst / "points3D.txt", "w") as f:
        for pid, xyz, rest in points:
            p = scale * rot @ xyz + offset
            f.write(f"{pid} {p[0]} {p[1]} {p[2]} {' '.join(rest)}\n")


def rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotação que leva o vetor unitário a até b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -0.999999:
        axis = np.cross(a, [1.0, 0, 0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0, 1.0, 0])
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def metric_alignment(images: dict, frames: list[dict]) -> tuple[float, np.ndarray, dict]:
    """
    Calcula a escala métrica e a direção "para cima" do modelo do COLMAP usando
    só os trechos em que o ARCore se comportou bem:
      - escala: razão entre deslocamentos curtos do ARCore e do COLMAP (mediana);
      - para cima: gravidade do ARCore (vem do acelerômetro, é confiável mesmo
        quando a posição se perde).
    """
    by_name = {Path(f["image"]).name: f for f in frames}
    reg = []
    for img in images.values():
        fr = by_name.get(img["name"])
        if fr:
            reg.append((fr["index"], fr, img))
    reg.sort(key=lambda x: x[0])

    ratios = []
    for k in range(len(reg)):
        for j in range(k + 1, min(k + 6, len(reg))):
            fa, fb = reg[k][1], reg[j][1]
            dt = (fb["timestamp_ns"] - fa["timestamp_ns"]) / 1e9
            da = float(np.linalg.norm(np.subtract(fb["t"], fa["t"])))
            dc = float(np.linalg.norm(camera_center(reg[j][2]) - camera_center(reg[k][2])))
            if dt <= 0 or dc <= 1e-9:
                continue
            if 0.1 <= da <= 1.5 and da / dt <= 1.0:
                ratios.append(da / dc)

    info = {"pares_para_escala": len(ratios)}
    scale = 1.0
    if len(ratios) >= 10:
        r = np.array(ratios)
        med = float(np.median(r))
        good = r[np.abs(r - med) < 0.25 * med]
        if len(good) >= 10:
            scale = float(np.median(good))
            info["dispersao_escala_pct"] = round(float(np.std(good) / scale * 100), 1)
    info["escala_confiavel"] = scale != 1.0

    ups = []
    for _, fr, img in reg:
        r_c2w_ar = quat_to_rot(*fr["q_xyzw"]) @ GL_TO_CV
        qw, qx, qy, qz = img["q"]
        r_c2w_col = quat_to_rot(qx, qy, qz, qw).T
        # "Para cima" no sistema do COLMAP visto por esta câmera.
        up_cam = r_c2w_ar.T @ np.array([0.0, 1.0, 0.0])
        ups.append(r_c2w_col @ up_cam)
    up = np.median(np.array(ups), axis=0) if ups else np.array([0.0, 1.0, 0.0])
    if np.linalg.norm(up) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
    return scale, rotation_between(up, np.array([0.0, 1.0, 0.0])), info


# ---------------------------------------------------------------------- COLMAP

def write_known_model(db_path: Path, frames: list[dict], out: Path, scale: float,
                      ref_path: Path) -> None:
    """Escreve um modelo COLMAP em texto com as poses do ARCore."""
    con = sqlite3.connect(db_path)
    ids = {name: (iid, cid) for iid, name, cid in con.execute("SELECT image_id, name, camera_id FROM images")}
    cams = list(con.execute("SELECT camera_id, width, height FROM cameras"))
    con.close()

    fx = float(np.median([f["fx"] for f in frames]))
    fy = float(np.median([f["fy"] for f in frames]))
    cx = float(np.median([f["cx"] for f in frames]))
    cy = float(np.median([f["cy"] for f in frames]))

    out.mkdir(parents=True, exist_ok=True)
    with open(out / "cameras.txt", "w") as f:
        for cid, w, h in cams:
            f.write(f"{cid} PINHOLE {w} {h} {fx} {fy} {cx} {cy}\n")
    with open(out / "images.txt", "w") as f, open(ref_path, "w") as ref:
        for fr in frames:
            name = Path(fr["image"]).name
            if name not in ids:
                continue
            iid, cid = ids[name]
            (qw, qx, qy, qz), tvec, center = arcore_to_colmap(fr["t"], fr["q_xyzw"], scale)
            f.write(f"{iid} {qw} {qx} {qy} {qz} {tvec[0]} {tvec[1]} {tvec[2]} {cid} {name}\n\n")
            ref.write(f"{name} {center[0]} {center[1]} {center[2]}\n")
    (out / "points3D.txt").write_text("")


def count_registered(colmap: str, model: Path, tmp: Path) -> int:
    tmp.mkdir(parents=True, exist_ok=True)
    run([colmap, "model_converter", "--input_path", model, "--output_path", tmp, "--output_type", "TXT"])
    lines = [l for l in (tmp / "images.txt").read_text().splitlines() if l and not l.startswith("#")]
    return len(lines) // 2


def extract_and_match(colmap: str, db: Path, images: Path, frames: list[dict]) -> None:
    fx = np.median([f["fx"] for f in frames]); fy = np.median([f["fy"] for f in frames])
    cx = np.median([f["cx"] for f in frames]); cy = np.median([f["cy"] for f in frames])

    log("COLMAP 1/5: extraindo pontos de interesse das fotos")
    run([colmap, "feature_extractor",
         "--database_path", db, "--image_path", images,
         "--ImageReader.camera_model", "PINHOLE",
         "--ImageReader.single_camera", "1",
         "--ImageReader.camera_params", f"{fx},{fy},{cx},{cy}"])

    log(f"COLMAP 2/5: comparando as {len(frames)} fotos entre si (é a etapa mais longa)")
    if len(frames) <= 1000:
        run([colmap, "exhaustive_matcher", "--database_path", db])
    else:
        run([colmap, "sequential_matcher", "--database_path", db,
             "--SequentialMatching.overlap", "50"])


def run_colmap(cap: Path, frames: list[dict], work: Path, scale: float,
               use_arcore_poses: bool, use_ai: bool = True,
               reuse: bool = False) -> tuple[Path, dict]:
    colmap = find_tool("colmap")
    cdir = work / "colmap"
    reuse = reuse and not use_arcore_poses and (cdir / "sparse").is_dir()
    if reuse:
        log("Reaproveitando a comparação de fotos e as posições já calculadas")
        for old in list(cdir.glob("modelo_txt*")) + [cdir / "modelo_metrico"]:
            shutil.rmtree(old, ignore_errors=True)
    else:
        if cdir.exists():
            shutil.rmtree(cdir)
        cdir.mkdir(parents=True)
    images = cap / "images"
    db = cdir / "database.db"
    info: dict = {}

    if not reuse:
        extract_and_match(colmap, db, images, frames)

    if use_arcore_poses:
        log("COLMAP 3/5: triangulando com as poses do ARCore")
        known = cdir / "sparse_arcore"
        write_known_model(db, frames, known, scale, cdir / "posicoes_arcore.txt")
        model = cdir / "sparse_tri"
        model.mkdir()
        run([colmap, "point_triangulator", "--database_path", db, "--image_path", images,
             "--input_path", known, "--output_path", model])
        txt = cdir / "modelo_txt"
        txt.mkdir()
        run([colmap, "model_converter", "--input_path", model, "--output_path", txt,
             "--output_type", "TXT"])
        final = txt
    else:
        if not reuse:
            log("COLMAP 3/5: calculando a posição de cada foto (fotogrametria)")
        sparse = cdir / "sparse"
        if not reuse:
            sparse.mkdir()
            run([colmap, "mapper", "--database_path", db, "--image_path", images,
                 "--output_path", sparse])
        # O mapper pode gerar mais de um pedaço; usa o que tem mais fotos.
        best, best_n, raw_txt = None, -1, None
        for sub in sorted(p for p in sparse.iterdir() if p.is_dir()):
            txt = cdir / f"modelo_txt_{sub.name}"
            txt.mkdir()
            run([colmap, "model_converter", "--input_path", sub, "--output_path", txt,
                 "--output_type", "TXT"])
            n = len(read_txt_model(txt)[0])
            log(f"  pedaço {sub.name}: {n} fotos")
            if n > best_n:
                best, best_n, raw_txt = sub, n, txt
        if raw_txt is None:
            raise SystemExit("O COLMAP não conseguiu montar o modelo. As fotos têm pouca sobreposição ou estão borradas.")
        info["pedacos"] = len([p for p in sparse.iterdir() if p.is_dir()])

        log("COLMAP 4/5: escala métrica e nivelamento pelo ARCore")
        imgs, pts = read_txt_model(raw_txt)
        s_metric, rot, align = metric_alignment(imgs, frames)
        info.update(align)
        total = s_metric * scale
        if not align["escala_confiavel"]:
            log("  [aviso] O ARCore não teve trechos confiáveis: a escala do modelo é arbitrária.")
            log("          Corrija no Blender medindo uma distância conhecida.")
        else:
            log(f"  escala calculada com {align['pares_para_escala']} pares de fotos "
                f"(variação {align.get('dispersao_escala_pct', '?')}%)")
        # Centraliza no meio do cômodo, com o ponto mais baixo (chão) em Y = 0.
        if pts:
            xyz = np.array([total * rot @ p for _, p, _ in pts])
            center = np.median(xyz, axis=0)
            floor = np.percentile(xyz[:, 1], 2)
            offset = np.array([-center[0], -floor, -center[2]])
        else:
            offset = np.zeros(3)
        final = cdir / "modelo_metrico"
        write_transformed_model(raw_txt, final, total, rot, offset)
        info["escala_fonte"] = "arcore" if align["escala_confiavel"] else "nenhuma"

        if use_ai:
            import escala_ia
            if escala_ia.available():
                log("  Conferindo a escala com a IA de profundidade métrica")
                try:
                    k, ai_info = escala_ia.estimate_scale(final, images, read_txt_model, quat_to_rot, log=log)
                except Exception as e:  # a IA é opcional: nunca derruba o processamento
                    k, ai_info = None, {"erro_ia": str(e)}
                info.update(ai_info)
                if k:
                    log(f"  IA: o modelo precisa ser multiplicado por {k:.3f} "
                        f"(variação entre fotos {ai_info.get('dispersao_ia_pct', '?')}%)")
                    total *= k
                    offset = offset * k
                    write_transformed_model(raw_txt, final, total, rot, offset)
                    info["escala_fonte"] = "ia"
                else:
                    log("  [aviso] A IA não teve fotos suficientes; mantendo a escala anterior.")
            else:
                log("  (IA de escala não instalada: rode instalar_ia.bat para ativar)")
        info["fator_escala_total"] = total

    registered = len(read_txt_model(final)[0])
    info["fotos_no_modelo"] = registered
    log(f"  {registered} de {len(frames)} fotos usadas no modelo")

    log("COLMAP 5/5: corrigindo distorção das fotos para a etapa densa")
    dense = work / "denso"
    if dense.exists():
        shutil.rmtree(dense)
    run([colmap, "image_undistorter", "--image_path", images, "--input_path", final,
         "--output_path", dense, "--output_type", "COLMAP"])
    return dense, info


# --------------------------------------------------------------------- OpenMVS

def texture_mesh(texture: str, scene: Path, mesh: Path, mvs: Path, decimate: float) -> None:
    out = mvs / "modelo.obj"
    base = [texture, "-i", scene, "-m", mesh, "-o", out, "-w", mvs, "--export-type", "obj"]
    if decimate < 1 and run(base + ["--decimate", str(decimate)], cwd=mvs, allow_fail=True):
        return
    run(base, cwd=mvs)


def run_dense_gpu(dense: Path, work: Path, detalhe: int) -> Path | None:
    """
    Nuvem densa e malha com o COLMAP (CUDA, placa de vídeo) e textura com o OpenMVS.
    Devolve a pasta com o resultado, ou None se alguma etapa do COLMAP falhar.
    """
    colmap = find_tool("colmap")
    mvs = work / "mvs"
    if mvs.exists():
        shutil.rmtree(mvs)
    mvs.mkdir()
    max_size = {0: 3000, 1: 2000, 2: 1000}[detalhe]

    log("Denso 1/4: profundidade de cada foto (placa de vídeo, é a etapa mais longa)")
    if not run([colmap, "patch_match_stereo", "--workspace_path", dense,
                "--workspace_format", "COLMAP",
                "--PatchMatchStereo.max_image_size", str(max_size),
                "--PatchMatchStereo.geom_consistency", "true"], allow_fail=True):
        return None

    log("Denso 2/4: juntando tudo numa nuvem de pontos")
    fused = dense / "fused.ply"
    if not run([colmap, "stereo_fusion", "--workspace_path", dense,
                "--workspace_format", "COLMAP", "--input_type", "geometric",
                "--output_path", fused], allow_fail=True):
        return None
    shutil.copy2(fused, mvs / "cena_densa.ply")

    log("Denso 3/4: gerando a malha (Delaunay, não inventa superfícies)")
    mesh = dense / "malha.ply"
    if not run([colmap, "delaunay_mesher", "--input_path", dense, "--input_type", "dense",
                "--output_path", mesh], allow_fail=True):
        return None

    log("Denso 4/4: aplicando textura das fotos")
    iface = find_tool("InterfaceCOLMAP")
    texture = find_tool("TextureMesh")
    run([iface, "-i", dense, "-o", mvs / "cena.mvs", "--image-folder", dense / "images"], cwd=mvs)
    texture_mesh(texture, mvs / "cena.mvs", mesh, mvs, {0: 1.0, 1: 0.5, 2: 0.25}[detalhe])
    return mvs


def run_openmvs(dense: Path, work: Path, resolution_level: int) -> Path:
    """Caminho alternativo, todo no OpenMVS (a nuvem densa roda na CPU)."""
    mvs = work / "mvs"
    mvs.mkdir(exist_ok=True)
    iface = find_tool("InterfaceCOLMAP")
    densify = find_tool("DensifyPointCloud")
    recon = find_tool("ReconstructMesh")
    texture = find_tool("TextureMesh")

    log("OpenMVS 1/4: importando cena")
    run([iface, "-i", dense, "-o", mvs / "cena.mvs", "--image-folder", dense / "images"], cwd=mvs)

    log("OpenMVS 2/4: nuvem de pontos densa (CPU, pode demorar)")
    run([densify, "-i", mvs / "cena.mvs", "-o", mvs / "cena_densa.mvs", "-w", mvs,
         "--resolution-level", str(resolution_level)], cwd=mvs)

    log("OpenMVS 3/4: gerando a malha (sem fechar buracos)")
    run([recon, "-i", mvs / "cena_densa.mvs", "-o", mvs / "malha.mvs", "-w", mvs,
         "--close-holes", "0"], cwd=mvs)

    log("OpenMVS 4/4: aplicando textura das fotos")
    texture_mesh(texture, mvs / "cena_densa.mvs", mvs / "malha.ply", mvs,
                 {0: 1.0, 1: 0.5, 2: 0.25}[resolution_level])
    return mvs


# -------------------------------------------------------------------- saída

def export_e57(ply: Path, out: Path) -> None:
    from plyfile import PlyData
    import pye57

    v = PlyData.read(str(ply))["vertex"]
    names = v.data.dtype.names
    data = {
        "cartesianX": np.asarray(v["x"], dtype=np.float64),
        "cartesianY": np.asarray(v["y"], dtype=np.float64),
        "cartesianZ": np.asarray(v["z"], dtype=np.float64),
    }
    if all(c in names for c in ("red", "green", "blue")):
        data["colorRed"] = np.asarray(v["red"], dtype=np.uint8)
        data["colorGreen"] = np.asarray(v["green"], dtype=np.uint8)
        data["colorBlue"] = np.asarray(v["blue"], dtype=np.uint8)
    e57 = pye57.E57(str(out), mode="w")
    e57.write_scan_raw(data)
    e57.close()


def collect(mvs: Path, dest: Path, want_e57: bool) -> list[str]:
    dest.mkdir(parents=True, exist_ok=True)
    produced = []
    for f in mvs.iterdir():
        if f.name.startswith("modelo") and f.suffix.lower() in (".obj", ".mtl", ".png", ".jpg"):
            shutil.copy2(f, dest / f.name)
            produced.append(f.name)
    dense_ply = mvs / "cena_densa.ply"
    if dense_ply.is_file():
        shutil.copy2(dense_ply, dest / "nuvem_densa.ply")
        produced.append("nuvem_densa.ply")
        if want_e57:
            try:
                log("Exportando nuvem de pontos E57")
                export_e57(dense_ply, dest / "nuvem_densa.e57")
                produced.append("nuvem_densa.e57")
            except ImportError:
                log("  E57 indisponível: a biblioteca pye57 não está instalada neste Python.")
    return produced


def main() -> None:
    ap = argparse.ArgumentParser(description="Gera o modelo 3D a partir de uma captura do Scan3D.")
    ap.add_argument("captura", type=Path, help=".zip exportado pelo app ou pasta da captura")
    ap.add_argument("--e57", action="store_true", help="também exporta a nuvem de pontos em E57")
    ap.add_argument("--usar-poses-arcore", action="store_true",
                    help="usa as posições do ARCore direto (mais rápido, só se o rastreamento foi bom)")
    ap.add_argument("--so-denso", action="store_true",
                    help="reaproveita as posições de um processamento anterior e refaz só a parte densa")
    ap.add_argument("--reaproveitar", action="store_true",
                    help="reaproveita a comparação de fotos e as posições de um processamento anterior")
    ap.add_argument("--sem-ia", action="store_true",
                    help="não usa a IA de profundidade para conferir a escala")
    ap.add_argument("--cpu", action="store_true",
                    help="usa o OpenMVS na CPU para a nuvem densa (mais lento)")
    ap.add_argument("--escala", type=float, default=1.0,
                    help="multiplica o tamanho do modelo (para corrigir a escala com uma medida conhecida)")
    ap.add_argument("--detalhe", type=int, default=1, choices=[0, 1, 2],
                    help="0 = máximo (mais lento), 1 = padrão, 2 = rápido")
    args = ap.parse_args()

    src = args.captura.resolve()
    name = src.stem if src.is_file() else src.name
    work = ROOT / "trabalho" / name
    work.mkdir(parents=True, exist_ok=True)
    start = time.time()

    cap = open_capture(src, work)
    frames = load_frames(cap)
    if len(frames) < 20:
        raise SystemExit(f"Só {len(frames)} fotos na captura. Faça uma captura mais longa.")
    log(f"{len(frames)} fotos encontradas")

    scale, calib = scale_from_calibration(cap)
    for m in calib:
        status = "usada" if m["usada"] else "DESCARTADA (ARCore estava perdido)"
        log(f"Medida de trena: real {m['real_cm']} cm | ARCore {m['arcore_cm']} cm | {status}")
    if scale != 1.0:
        log(f"Correção de escala pela trena: fator {scale:.4f}")

    if args.so_denso and (work / "denso" / "sparse").is_dir():
        log("Reaproveitando as posições já calculadas (pulando o COLMAP)")
        dense = work / "denso"
        old = ROOT / "resultados" / name / "relatorio.json"
        colmap_info = json.loads(old.read_text(encoding="utf-8")).get("colmap", {}) if old.is_file() else {}
        # Limpa resultados densos anteriores do COLMAP, se houver.
        shutil.rmtree(dense / "stereo" / "depth_maps", ignore_errors=True)
        shutil.rmtree(dense / "stereo" / "normal_maps", ignore_errors=True)
        (dense / "stereo" / "depth_maps").mkdir(parents=True, exist_ok=True)
        (dense / "stereo" / "normal_maps").mkdir(parents=True, exist_ok=True)
    else:
        dense, colmap_info = run_colmap(cap, frames, work, scale * args.escala,
                                        args.usar_poses_arcore, not args.sem_ia,
                                        reuse=args.reaproveitar)
    mvs = None if args.cpu else run_dense_gpu(dense, work, args.detalhe)
    if mvs is None:
        if not args.cpu:
            log("[aviso] Etapa densa na placa de vídeo falhou; usando o OpenMVS na CPU.")
        mvs = run_openmvs(dense, work, args.detalhe)

    dest = ROOT / "resultados" / name
    produced = collect(mvs, dest, args.e57)
    minutes = (time.time() - start) / 60
    report = {
        "captura": name,
        "fotos": len(frames),
        "correcao_trena": scale,
        "colmap": colmap_info,
        "medidas_trena": calib,
        "arquivos": produced,
        "minutos": round(minutes, 1),
        "unidade": "metros",
        "eixos": "Y para cima (o Blender converte para Z para cima ao importar OBJ)",
    }
    (dest / "relatorio.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    log(f"Pronto em {minutes:.0f} min. Resultado em: {dest}")
    log("No Blender: File > Import > Wavefront (.obj) e escolha modelo.obj")


if __name__ == "__main__":
    main()
