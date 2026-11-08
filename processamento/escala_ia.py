"""
Escala métrica por IA (Depth Anything V2 Metric, roda local na placa de vídeo).

A IA estima, para cada foto, a distância em metros de cada pixel. O COLMAP
sabe a distância (em unidades arbitrárias) dos seus pontos 3D até cada foto.
A razão entre as duas, tirada a mediana em dezenas de fotos e milhares de
pontos, dá o fator que leva o modelo para metros. A IA não cria geometria:
só serve de régua.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf"


def available() -> bool:
    try:
        import torch  # noqa: F401
        import transformers  # noqa: F401
        return True
    except ImportError:
        return False


def _upright_turns(r_w2c: np.ndarray) -> int:
    """Quantos giros de 90° (np.rot90) deixam a foto em pé, usando o "para cima" do modelo nivelado."""
    up = r_w2c @ np.array([0.0, 1.0, 0.0])  # para cima, em coordenadas da câmera (x dir., y baixo)
    ux, uy = up[0], -up[1]                   # no plano da imagem, com y para cima
    if abs(uy) >= abs(ux):
        return 0 if uy > 0 else 2
    # "para cima" aponta para a direita: girar 90° anti-horário; para a esquerda: horário.
    return 1 if ux > 0 else 3


def estimate_scale(model_dir: Path, image_dir: Path, read_txt_model, quat_to_rot,
                   max_images: int = 60, log=print) -> tuple[float | None, dict]:
    import torch
    from PIL import Image
    from transformers import pipeline

    images, points = read_txt_model(model_dir)
    pts = {int(pid): xyz for pid, xyz, _ in points}
    ids = sorted(images)
    if len(ids) > max_images:
        ids = [ids[i] for i in np.linspace(0, len(ids) - 1, max_images).astype(int)]

    device = 0 if torch.cuda.is_available() else -1
    log(f"  IA de profundidade: {MODEL_ID} ({'placa de vídeo' if device == 0 else 'CPU'})")
    pipe = pipeline("depth-estimation", model=MODEL_ID, device=device)

    per_image = []
    for iid in ids:
        img = images[iid]
        qw, qx, qy, qz = img["q"]
        r = quat_to_rot(qx, qy, qz, qw)
        obs = img["pts"].split()
        uv, z = [], []
        for k in range(0, len(obs) - 2, 3):
            pid = int(obs[k + 2])
            if pid < 0 or pid not in pts:
                continue
            pc = r @ pts[pid] + img["t"]
            if pc[2] <= 0:
                continue
            uv.append((float(obs[k]), float(obs[k + 1])))
            z.append(pc[2])
        if len(z) < 30:
            continue

        photo = Image.open(image_dir / img["name"]).convert("RGB")
        w, h = photo.size
        turns = _upright_turns(r)
        upright = Image.fromarray(np.rot90(np.asarray(photo), turns).copy())
        with torch.no_grad():
            depth = pipe(upright)["predicted_depth"]
        depth = torch.nn.functional.interpolate(
            depth.reshape(1, 1, *depth.shape[-2:]).float(),
            size=(upright.size[1], upright.size[0]), mode="bilinear", align_corners=False,
        )[0, 0].cpu().numpy()
        depth = np.rot90(depth, -turns)  # volta para a orientação original da foto
        if depth.shape != (h, w):
            continue

        uv = np.array(uv)
        cols = np.clip(uv[:, 0].astype(int), 0, w - 1)
        rows = np.clip(uv[:, 1].astype(int), 0, h - 1)
        metric = depth[rows, cols]
        ok = (metric > 0.2) & (metric < 15)
        if ok.sum() < 30:
            continue
        per_image.append(float(np.median(metric[ok] / np.asarray(z)[ok])))

    if len(per_image) < 5:
        return None, {"fotos_ia": len(per_image)}
    vals = np.array(per_image)
    med = float(np.median(vals))
    good = vals[np.abs(vals - med) < 0.3 * med]
    scale = float(np.median(good))
    return scale, {
        "fotos_ia": len(per_image),
        "dispersao_ia_pct": round(float(np.std(good) / scale * 100), 1),
    }
