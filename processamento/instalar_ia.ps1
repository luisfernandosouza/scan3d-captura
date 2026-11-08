# Instala a IA de escala metrica (PyTorch com CUDA + Depth Anything V2 Metric).
# Rode com: instalar_ia.bat (duplo clique). Precisa de uns 5 GB livres.
$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$py = (py -3 -c "import sys; print(sys.executable)").Trim()
Write-Host "Python: $py"

Write-Host "[..] Instalando PyTorch com suporte a placa NVIDIA (download grande, pode demorar)"
$ok = $false
foreach ($cu in @("cu128", "cu126", "cu124")) {
    & $py -m pip install --upgrade torch --index-url "https://download.pytorch.org/whl/$cu"
    if ($LASTEXITCODE -eq 0) { $ok = $true; break }
}
if (-not $ok) {
    Write-Host "[erro] Nao foi possivel instalar o PyTorch com CUDA para esta versao do Python."
    exit 1
}
& $py -m pip install --upgrade transformers pillow
if ($LASTEXITCODE -ne 0) { Write-Host "[erro] Falha ao instalar transformers."; exit 1 }

Write-Host "[..] Baixando o modelo de IA (cerca de 1,3 GB)"
& $py -c "import torch; from transformers import pipeline; print('CUDA disponivel:', torch.cuda.is_available()); pipeline('depth-estimation', model='depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf')"
if ($LASTEXITCODE -ne 0) { Write-Host "[erro] Falha ao baixar o modelo."; exit 1 }

Write-Host ""
Write-Host "IA instalada. O processamento passa a usa-la automaticamente para a escala."
