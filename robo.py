"""
Simulação do UFactory Lite 6 no MuJoCo: célula de aplicação de selante em PCB.

Sequência:
  1. Carrega a cena (robô + bancada + PCB) e abre o visualizador.
  2. Move cada junta sozinha: repouso -> mínimo -> máximo -> repouso.
  3. Movimento coordenado: o robô sai do repouso, desce sobre a PCB, percorre
     os 4 cantos da trilha de selante (laranja) e volta ao repouso.

Como rodar:  py -3.11 robo.py   (Windows)
             python3 robo.py    (Linux)
             mjpython robo.py   (macOS)
"""
import math
import os
import time

import mujoco
import mujoco.viewer

# ---------- 1. Carregar o modelo ----------
# Entra na pasta deste arquivo e abre a cena por caminho relativo.
# Funciona mesmo se o terminal estiver em outra pasta, e evita um problema do
# MuJoCo no Windows: ele não abre caminhos completos com acentos (ex.: "Área").
os.chdir(os.path.dirname(os.path.abspath(__file__)))
model = mujoco.MjModel.from_xml_path("ufactory_lite6/scene.xml")
data = mujoco.MjData(model)

# ---------- Poses (ângulos das 6 juntas, em radianos) ----------
# Repouso: braço erguido, antebraço na horizontal, flange apontando para baixo
REPOUSO = [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0]

# Aproximação: flange 10 cm acima do centro da PCB
ACIMA_PCB = [0.0, 0.83, 0.95, 0.0, 0.12, 0.0]

# Cantos da trilha de selante, com o flange 6 cm acima da placa.
# A junta 1 gira a base para a esquerda (+) ou para a direita (-).
# A junta 5 vale (junta 3 - junta 2), o que mantém o flange na vertical.
CANTO_1 = [0.07, 1.08, 1.20, 0.0, 0.12, 0.0]    # frente, esquerda
CANTO_2 = [0.09, 0.97, 0.88, 0.0, -0.09, 0.0]   # trás,   esquerda
CANTO_3 = [-0.09, 0.97, 0.88, 0.0, -0.09, 0.0]  # trás,   direita
CANTO_4 = [-0.07, 1.08, 1.20, 0.0, 0.12, 0.0]   # frente, direita

# ---------- Faixa de varredura de cada junta (graus) ----------
# Juntas 1, 4 e 6 giram o limite completo.
# Juntas 2, 3 e 5 usam uma faixa menor: perto do limite real o braço
# colide com ele mesmo ou com a bancada.
FAIXA = [
    (-360, 360),  # junta 1 - base
    (-130, 75),   # junta 2 - ombro
    (10, 285),    # junta 3 - cotovelo
    (-360, 360),  # junta 4 - giro do antebraço
    (-90, 90),    # junta 5 - punho
    (-360, 360),  # junta 6 - flange
]

VEL_JUNTAS = 1.5   # rad/s nas varreduras individuais
VEL_TAREFA = 0.4   # rad/s no movimento coordenado (mais lento, para ficar visível)
DT = 0.01          # intervalo entre quadros (s)


def aplicar(q):
    """Coloca o robô na posição q e avança um passo da simulação."""
    data.qpos[:] = q   # ângulos das juntas
    data.ctrl[:] = q   # alvo dos motores (servos de posição do modelo)
    mujoco.mj_step(model, data)


def mover(viewer, q_inicio, q_fim, velocidade):
    """Leva o robô de q_inicio a q_fim, mexendo todas as juntas ao mesmo tempo."""
    maior_dist = max(abs(b - a) for a, b in zip(q_inicio, q_fim))
    passos = max(1, int(maior_dist / velocidade / DT))

    for i in range(passos + 1):
        if not viewer.is_running():
            return
        t = i / passos
        s = t * t * (3 - 2 * t)  # suavização: começa e termina devagar
        q = [a + s * (b - a) for a, b in zip(q_inicio, q_fim)]
        aplicar(q)
        viewer.sync()
        time.sleep(DT)


def esperar(viewer, q, segundos):
    """Mantém o robô parado na posição q."""
    for _ in range(int(segundos / DT)):
        if not viewer.is_running():
            return
        aplicar(q)
        viewer.sync()
        time.sleep(DT)


def posicao_flange():
    """Posição (x, y, z) do flange em milímetros, calculada pelo MuJoCo."""
    x, y, z = data.body("link6").xpos
    return f"x={x * 1000:4.0f} mm  y={y * 1000:4.0f} mm  z={z * 1000:4.0f} mm"


# Começa na pose de repouso
data.qpos[:] = REPOUSO
data.ctrl[:] = REPOUSO
mujoco.mj_forward(model, data)

# Painel da direita (dados das juntas) visível; o da esquerda fica escondido.
# Tab mostra/esconde o painel esquerdo e Shift+Tab o direito.
with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=True) as viewer:
    # Câmera enquadrando o robô e a PCB
    viewer.cam.lookat[:] = [0.15, 0.0, 0.15]
    viewer.cam.distance = 1.0
    viewer.cam.azimuth = 140
    viewer.cam.elevation = -25

    print("Repouso. Flange em", posicao_flange())
    esperar(viewer, REPOUSO, 3.0)

    # ---------- 2. Cada junta individualmente ----------
    print("\n=== Juntas individuais ===")
    for j in range(model.njnt):
        nome = model.joint(j).name
        lim_min, lim_max = model.jnt_range[j]
        minimo = math.radians(FAIXA[j][0])
        maximo = math.radians(FAIXA[j][1])
        print(f"{nome}: limite real {math.degrees(lim_min):5.0f}° a {math.degrees(lim_max):4.0f}°"
              f"  |  varrendo {FAIXA[j][0]:5}° a {FAIXA[j][1]:4}°")

        q_min = list(REPOUSO)
        q_min[j] = minimo
        q_max = list(REPOUSO)
        q_max[j] = maximo

        mover(viewer, REPOUSO, q_min, VEL_JUNTAS)
        mover(viewer, q_min, q_max, VEL_JUNTAS)
        mover(viewer, q_max, REPOUSO, VEL_JUNTAS)
        esperar(viewer, REPOUSO, 0.5)

    # ---------- 3. Movimento coordenado: contorno da trilha de selante ----------
    print("\n=== Movimento coordenado ===")
    trajeto = [
        ("Aproximação sobre a PCB", ACIMA_PCB),
        ("Canto 1", CANTO_1),
        ("Canto 2", CANTO_2),
        ("Canto 3", CANTO_3),
        ("Canto 4", CANTO_4),
        ("Canto 1 (fecha o contorno)", CANTO_1),
        ("Afastamento", ACIMA_PCB),
        ("Repouso", REPOUSO),
    ]
    q_atual = REPOUSO
    for nome, pose in trajeto:
        mover(viewer, q_atual, pose, VEL_TAREFA)
        print(f"{nome:28} -> flange em {posicao_flange()}")
        esperar(viewer, pose, 0.5)
        q_atual = pose

    print("\nFim. Feche a janela para sair.")
    while viewer.is_running():
        esperar(viewer, REPOUSO, 0.1)
