# Simulação do UFactory Lite 6 no MuJoCo: célula de selante em PCB

Simulação de um braço robótico **UFactory Lite 6** (6 graus de liberdade) em uma bancada de
montagem eletrônica, onde o robô aplica selante em uma placa de circuito impresso (PCB).

O script:

1. move cada junta individualmente, mostrando sua faixa de movimento;
2. faz um movimento coordenado em que o robô sai do repouso, desce sobre a PCB, percorre os
   4 cantos da trilha de selante e volta ao repouso.

| Item | Valor |
|---|---|
| Simulador | MuJoCo 3.3 ou mais recente (biblioteca Python `mujoco`) |
| Linguagem | Python 3.11 |
| Robô | UFactory Lite 6, modelo do [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_lite6) |
| Sistemas | Windows, Linux e macOS |
| Vídeo | [Simulação rodando (Google Drive)](https://drive.google.com/file/d/1V2ehtKp4JgyYJ_5_L4XqeUZYM9Il-e67/view?usp=sharing) |

---

## Estrutura de pastas

```
projeto/
├── robo.py              <- script principal
├── requirements.txt     <- dependência (mujoco)
├── README.md
└── ufactory_lite6/      <- pasta do MuJoCo Menagerie
    ├── lite6.xml        <- modelo do robô (original do Menagerie)
    ├── scene.xml        <- cena da célula (bancada, PCB, bandeja, controladora)
    └── assets/          <- malhas 3D do robô
```

O `scene.xml` precisa ficar **dentro** de `ufactory_lite6/`, ao lado de `lite6.xml`, porque o
modelo procura as malhas na pasta `assets/` a partir da pasta do arquivo de cena.

---

## Instalação

1. Instale o Python 3.11 (ou mais recente).
2. Na pasta do projeto, instale o MuJoCo:

   ```bash
   py -3.11 -m pip install -r requirements.txt
   ```

   No Linux ou macOS, use `python3 -m pip install -r requirements.txt`.

3. Confira a instalação:

   ```bash
   py -3.11 -c "import mujoco; print(mujoco.__version__)"
   ```

A pasta `ufactory_lite6/` já vem no projeto com a cena e os modelos. Não é preciso baixar o
Menagerie separadamente.

---

## Como rodar

| Sistema | Comando |
|---|---|
| Windows | `py -3.11 robo.py` |
| Linux | `python3 robo.py` |
| macOS | `mjpython robo.py` (no Mac, o visualizador do MuJoCo exige o `mjpython`) |

A janela do MuJoCo abre já enquadrando o robô e a PCB, e a sequência começa sozinha:

| Etapa | Duração aprox. | O que acontece |
|---|---|---|
| Repouso | 3 s | Robô parado na pose inicial |
| Juntas 1 a 6 | 65 s | Cada junta vai ao mínimo, depois ao máximo, e volta ao repouso |
| Movimento coordenado | 15 s | Aproximação, contorno dos 4 cantos da trilha de selante, afastamento e repouso |

O terminal mostra os limites de cada junta e a posição do flange (em mm) a cada ponto do
contorno. A janela fica aberta até ser fechada.

**Controles do visualizador:**

- arrastar com o botão esquerdo: girar a câmera;
- scroll: zoom;
- duplo clique em uma peça: centralizar nela;
- `Tab` / `Shift+Tab`: mostrar ou esconder os painéis laterais.

### Problemas comuns

| Erro | Solução |
|---|---|
| `ModuleNotFoundError: No module named 'mujoco'` | Instale com o mesmo Python que roda o script: `py -3.11 -m pip install mujoco`. |
| Erro dizendo que não encontrou `scene.xml` ou um arquivo `.stl` | Confira se a pasta `ufactory_lite6/` está ao lado do `robo.py` e se o `scene.xml` está dentro dela. |
| `TypeError` em `launch_passive` | Versão antiga do MuJoCo. Atualize com `py -3.11 -m pip install -U mujoco`. |
| macOS: erro pedindo para rodar com `mjpython` | Rode com `mjpython robo.py`. |

---

## O robô

| Junta | Função | Tipo | Limite real | Faixa usada na demo |
|---|---|---|---|---|
| 1 | Base (giro em torno do eixo vertical) | R | -360° a +360° | -360° a +360° |
| 2 | Ombro | R | -150° a +150° | -130° a +75° |
| 3 | Cotovelo | R | -3,5° a +300° | +10° a +285° |
| 4 | Giro do antebraço | R | -360° a +360° | -360° a +360° |
| 5 | Punho (flexão) | R | -124° a +124° | -90° a +90° |
| 6 | Flange (giro da ferramenta) | R | -360° a +360° | -360° a +360° |

- **6 graus de liberdade**, todos de juntas rotativas (configuração **6R**).
- Alcance de cerca de **440 mm**.
- **Faixas reduzidas nas juntas 2, 3 e 5:** perto do limite real, o braço colide com ele mesmo
  (antebraço contra o braço ou a base) ou com a bancada. As faixas da demo foram verificadas
  no MuJoCo sem nenhum contato entre peças, com cerca de 5° de margem.

## Como o movimento é feito

- As poses são listas com os 6 ângulos de junta, em radianos. O movimento é feito escrevendo
  diretamente em `data.qpos` (e em `data.ctrl`, o alvo dos servos do modelo), sem cinemática
  inversa.
- Entre duas poses, todas as juntas são interpoladas ao mesmo tempo com uma curva suave
  (*smoothstep*), de modo que o movimento começa e termina devagar.
- As poses dos cantos da PCB foram obtidas por cinemática direta. Nos cantos, a junta 5 vale
  (junta 3 − junta 2), o que mantém o flange apontando para baixo.

## Cenário (`scene.xml`)

- Bancada com tampo e quatro pernas, sobre o piso do laboratório.
- Gabarito preto com uma PCB de 100 × 70 mm, chips e uma trilha de selante (retângulo laranja
  de 60 × 40 mm) na frente do robô.
- Bandeja de placas e caixa da controladora ao redor do robô.
