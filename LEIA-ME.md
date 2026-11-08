# Scan3D Captura (passo 1)

App Android que captura fotos de um ambiente junto com a posição exata da câmera (ARCore). Esses dados vão para o processamento no seu PC (passo 2), que gera o modelo 3D em escala real.

## Como instalar no Galaxy A55

1. Instale o **Android Studio** no seu PC: https://developer.android.com/studio
2. Descompacte `scan3d-captura.zip` e, no Android Studio, use **File > Open** na pasta `scan3d`.
   - Se o Android Studio sugerir atualizar o Android Gradle Plugin, pode aceitar.
3. No celular, ative o **modo desenvolvedor**: Configurações > Sobre o telefone > Informações do software > toque 7 vezes em "Número de compilação".
   Depois, em Configurações > Opções do desenvolvedor, ative **Depuração USB**.
4. Ligue o celular ao PC pelo cabo USB e aceite a autorização na tela do celular.
5. No Android Studio, escolha o A55 na lista de aparelhos e clique em **Run** (▶).
6. Na primeira abertura, o app pede permissão da câmera e pode pedir para instalar o
   **Google Play Services para RA** (ARCore). Aceite.

## Como capturar bem

- **Acenda todas as luzes.** Pouca luz gera fotos borradas e perda de rastreamento.
- Toque em **Iniciar** e espere a mensagem "Gravando". O app tira as fotos sozinho: uma a cada 15 cm andados ou a cada 10° girados.
- **Ande, não gire parado.** A reconstrução precisa ver cada superfície de pontos diferentes. Percorra o cômodo rente às paredes, com a câmera voltada para o centro, e depois faça uma volta pelo meio olhando para as paredes.
- **Devagar.** Se aparecer "Mais devagar!", o app para de fotografar até você reduzir a velocidade.
- Siga o painel de cobertura: **Paredes** deve chegar a 100%, e **Teto** e **Piso** devem ter fotos (aponte o celular para cima e para baixo).
- Capture de perto os cantos, encontros de paredes, batentes, janelas e degraus.
- Superfícies lisas e brancas, vidros e espelhos são difíceis. Passe por elas devagar e em ângulo.

## Medida de referência (opcional, recomendada para levantamento técnico)

Durante a gravação:
1. Meça com a trena uma distância bem definida, como a largura de uma porta ou o comprimento de uma parede. Quanto maior a distância, melhor.
2. Toque em **Medir**, depois toque nos dois pontos na tela.
3. O app mostra a medida estimada pelo ARCore. Digite a medida real em cm.

Faça de 2 a 3 medidas em direções diferentes. O passo 2 usa essas medidas para corrigir a escala do modelo inteiro.

## Como levar a captura para o PC

Toque em **Exportar** depois de parar a gravação. O app gera um `.zip` que você pode enviar pelo Google Drive, Quick Share ou outro app.

Também dá para copiar pelo cabo USB, na pasta:
`Android/data/com.scan3d.captura/files/capturas/`

## O que cada captura contém

```
AAAAMMDD_HHMMSS/
  session.json       aparelho, resolução, convenções de coordenadas
  frames.jsonl       uma linha por foto: posição (t), rotação (q_xyzw), fx, fy, cx, cy
  images/*.jpg       fotos
  depth/*.u16        profundidade estimada pelo ARCore (uint16, mm), quando disponível
  calibration.json   medidas de trena (se você fez alguma)
```

As posições estão em metros, no sistema de coordenadas do ARCore (Y para cima).

## Limitações conhecidas desta versão

- O ARCore costuma entregar imagens de até cerca de 1920×1080, não os 50 MP da câmera. A resolução real do A55 fica registrada em `session.json`. Se a definição não for suficiente, uma versão futura pode adicionar um modo de fotos em alta resolução.
- A profundidade do A55 é estimada por software (o aparelho não tem sensor de profundidade). Ela serve como apoio, não como medida final.
- Se o app for minimizado durante a gravação, a captura é encerrada, porque o ARCore perde o sistema de coordenadas.
