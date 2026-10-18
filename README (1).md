# Simulação do UFactory Lite 6 no MuJoCo: célula de selante em PCB

Simulação de um braço robótico **UFactory Lite 6** (6 graus de liberdade) em uma bancada de
laboratório onde o robô aplica selante em uma placa de circuito impresso (PCB).

O script move cada junta individualmente, mostrando sua faixa de movimento. Depois faz um
movimento coordenado da pose de repouso até logo acima da PCB.

| Item | Valor |
|---|---|
| Simulador | MuJoCo (biblioteca Python `mujoco`) |
| Robô | UFactory Lite 6, modelo do [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_lite6) |
| Linguagem | Python 3.9 ou mais recente |
| Sistemas | Windows, Linux e macOS |

---

## Estrutura de pastas

```
projeto/
├── robo.py                  <- script principal
├── README.md
└── ufactory_lite6/          <- pasta copiada do MuJoCo Menagerie
    ├── lite6.xml            <- modelo do robô (original do Menagerie)
    ├── scene.xml            <- NOSSA cena (substitui o scene.xml original)
    └── assets/              <- malhas 3D do robô
```

> O `scene.xml` precisa ficar **dentro** de `ufactory_lite6/`, ao lado de `lite6.xml`.
> O modelo procura as malhas na pasta `assets/` a partir da pasta do arquivo de cena.

---

## Instalação

### 1. Instalar o MuJoCo

```bash
pip install mujoco
```

Para conferir a versão instalada (anote-a na tabela acima):

```bash
python -c "import mujoco; print(mujoco.__version__)"
```

### 2. Baixar o modelo do robô

Opção A, com Git:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

Opção B, sem Git: no GitHub do Menagerie, clique em **Code > Download ZIP** e extraia o arquivo.

### 3. Montar a pasta do projeto

1. Copie a pasta `mujoco_menagerie/ufactory_lite6/` para dentro da pasta do projeto.
2. Coloque o nosso `scene.xml` dentro de `ufactory_lite6/` e substitua o original.
3. Deixe o `robo.py` na pasta do projeto, ao lado de `ufactory_lite6/`.

---

## Como rodar

Abra o terminal **na pasta do projeto** e rode:

| Sistema | Comando |
|---|---|
| Windows / Linux | `python robo.py` |
| macOS | `mjpython robo.py` (o visualizador do MuJoCo exige o `mjpython` no Mac) |

A janela do MuJoCo abre e a sequência começa sozinha (cerca de 1 min e 20 s):

1. 2 s parado na pose de repouso.
2. Juntas 1 a 6, uma de cada vez: vai ao mínimo, depois ao máximo, e volta ao repouso.
   O terminal mostra o limite real de cada junta e a faixa que está sendo varrida.
3. Movimento coordenado até ficar acima da PCB, 2 s de pausa e volta ao repouso.
4. A janela fica aberta até você fechá-la.

Dicas do visualizador: arraste com o botão esquerdo para girar a câmera, use o scroll para dar
zoom e clique duas vezes em uma peça para centralizar nela.

### Problemas comuns

| Erro | Causa e solução |
|---|---|
| `ValueError` dizendo que não abriu `ufactory_lite6/scene.xml` | O terminal não está na pasta do projeto. Use `cd` até a pasta onde está o `robo.py`. |
| Erro de malha (`.stl`) não encontrada | O `scene.xml` está fora da pasta `ufactory_lite6/`. |
| macOS: erro pedindo para rodar com `mjpython` | Rode com `mjpython robo.py`. |
| `ModuleNotFoundError: No module named 'mujoco'` | Rode `pip install mujoco` no mesmo Python que executa o script. |

---

## Sobre o robô

| Junta | Função | Tipo | Limite real | Faixa usada na demo |
|---|---|---|---|---|
| 1 | Base (giro em torno do eixo vertical) | R | -360° a +360° | -360° a +360° |
| 2 | Ombro | R | -150° a +150° | -130° a +75° |
| 3 | Cotovelo | R | -3,5° a +300° | +10° a +285° |
| 4 | Giro do antebraço | R | -360° a +360° | -360° a +360° |
| 5 | Punho (flexão) | R | -124° a +124° | -90° a +90° |
| 6 | Flange (giro da ferramenta) | R | -360° a +360° | -360° a +360° |

São 6 graus de liberdade, todos de juntas rotativas (configuração 6R). O alcance é de cerca de 440 mm.

**Por que a faixa usada é menor nas juntas 2, 3 e 5:** perto do limite real, o braço colide com
ele mesmo (antebraço contra o braço ou a base) ou com a bancada. As faixas da demo foram
escolhidas checando colisões no próprio MuJoCo, com uma margem de cerca de 5°.

---

## Uso de IA

> Complete esta seção com a sua experiência. Os itens abaixo são um ponto de partida.

- **Ferramentas usadas:** Claude (Anthropic) para montar a cena, simplificar o script, achar
  ângulos de junta sem cinemática inversa e escrever este README.
- **Onde ajudou:** calculou uma pose de juntas que deixa a ferramenta logo acima da PCB usando
  só cinemática direta, e explicou por que é preciso escrever em `data.ctrl` além de
  `data.qpos` (os servos de posição do modelo puxariam o braço de volta).
- **Onde errou:** a primeira versão varria as juntas a partir da pose dobrada (todas as juntas
  em zero) e usava o limite completo. Ao rodar, as juntas 2, 3 e 4 atravessavam outras partes
  do robô. A correção foi mudar a pose de partida e limitar a faixa de varredura.
