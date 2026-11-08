# Processamento (passo 2)

Transforma uma captura do app em um modelo 3D texturizado, em escala real, para abrir no Blender.

Requisitos: Windows 10 ou 11 e placa NVIDIA (testado para uma RTX 3060 de 12 GB).

## Instalar (uma vez)

Dê dois cliques em `instalar.bat`. Ele baixa o COLMAP e o OpenMVS para a pasta `ferramentas`, e instala o Python (se faltar) e as bibliotecas necessárias.

## Processar uma captura

1. No app, toque em **Exportar** e envie o `.zip` para o PC.
2. Arraste o `.zip` para cima de `processar.bat`.
3. Espere terminar. O resultado fica em `resultados/<nome da captura>/`:
   - `modelo.obj` + `modelo.mtl` + texturas: o modelo para o Blender
   - `nuvem_densa.ply`: a nuvem de pontos
   - `relatorio.json`: fotos usadas, fator de escala e erro das medidas de trena

Para gerar também uma nuvem E57 (Revit/AutoCAD), rode no terminal:
`py -3 processar.py captura.zip --e57`

Opção `--detalhe`: `0` = máximo (mais lento), `1` = padrão, `2` = rápido para testes.

## Abrir no Blender

File > Import > Wavefront (.obj) e escolha `modelo.obj`. As unidades estão em metros e o Blender já ajusta os eixos.
Para ver a textura, mude a visualização da janela 3D para **Material Preview**.

## Como as posições são calculadas

O COLMAP calcula a posição de cada foto só a partir das próprias fotos (fotogrametria). O ARCore entra apenas para dar a escala em metros e para saber onde é "para cima", usando somente os trechos da captura em que o rastreamento dele estava estável. Se o ARCore não teve nenhum trecho confiável, o modelo sai em escala arbitrária: no Blender, meça uma distância conhecida e ajuste a escala.

Se o rastreamento foi bom do começo ao fim, `--usar-poses-arcore` pula o cálculo das posições e termina mais rápido.

## Fidelidade

- A malha é gerada sem fechar buracos: o que não foi fotografado fica vazio, nada é inventado.
- A escala vem do ARCore e, se houver medidas de trena, é corrigida pela mediana delas. Confira o erro restante em `relatorio.json`.
