# Instala as ferramentas do processamento (COLMAP + OpenMVS) e as bibliotecas Python.
# Rode com: instalar.bat (duplo clique)
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$tools = Join-Path $root "ferramentas"
New-Item -ItemType Directory -Force $tools | Out-Null

function Get-Asset($repo, $pattern) {
    $rel = Invoke-RestMethod "https://api.github.com/repos/$repo/releases/latest" -Headers @{ "User-Agent" = "scan3d-instalador" }
    $asset = $rel.assets | Where-Object { $_.name -match $pattern } | Select-Object -First 1
    if (-not $asset) {
        throw "Nao encontrei o pacote para Windows de $repo na versao $($rel.tag_name). Veja: $($rel.html_url)"
    }
    Write-Host "  versao $($rel.tag_name): $($asset.name)"
    return $asset
}

function Install-Package($name, $repo, $pattern, $exe) {
    $found = Get-ChildItem $tools -Recurse -Filter $exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($found) {
        Write-Host "[ok] $name ja instalado em $($found.DirectoryName)"
        return
    }
    Write-Host "[..] Baixando $name"
    $asset = Get-Asset $repo $pattern
    $file = Join-Path $env:TEMP $asset.name
    Invoke-WebRequest $asset.browser_download_url -OutFile $file
    $dest = Join-Path $tools $name
    New-Item -ItemType Directory -Force $dest | Out-Null
    if ($asset.name -like "*.zip") {
        Expand-Archive $file -DestinationPath $dest -Force
    } else {
        # O tar do Windows 10/11 abre .7z
        tar -xf $file -C $dest
    }
    Remove-Item $file
    $found = Get-ChildItem $dest -Recurse -Filter $exe -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $found) { throw "$name foi baixado, mas $exe nao apareceu em $dest" }
    Write-Host "[ok] $name instalado em $($found.DirectoryName)"
}

Install-Package "colmap" "colmap/colmap" "(?i)windows.*cuda.*\.zip$" "colmap.exe"
Install-Package "openmvs" "cdcseacave/openMVS" "(?i)win.*\.(zip|7z)$" "TextureMesh.exe"

Write-Host "[..] Verificando Python"
$py = Get-Command py -ErrorAction SilentlyContinue
if (-not $py) {
    Write-Host "Python nao encontrado. Instalando pelo winget..."
    winget install -e --id Python.Python.3.12 --accept-package-agreements --accept-source-agreements
    Write-Host "Feche esta janela e rode instalar.bat de novo para terminar."
    exit 1
}
py -3 -m pip install --upgrade -r (Join-Path $root "requirements.txt")

Write-Host ""
Write-Host "Tudo pronto. Para processar, arraste o .zip da captura para processar.bat"
