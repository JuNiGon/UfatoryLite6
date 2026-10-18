"""
Simulação do UFactory Lite 6 no MuJoCo - célula de selante em PCB.

O que o script faz:
  1. Carrega a cena (robô + bancada + PCB).
  2. Move cada junta sozinha: repouso -> mínimo -> máximo -> repouso.
  3. Faz um movimento coordenado (todas as juntas juntas) do repouso até
     ficar logo acima da PCB, espera um pouco e volta.

Como rodar:  python robo.py      (no macOS: mjpython robo.py)
"""
import math
import time

import mujoco
import mujoco.viewer

# ---------- 1. Carregar o modelo ----------
model = mujoco.MjModel.from_xml_path("ufactory_lite6/scene.xml")
data = mujoco.MjData(model)

# ---------- Poses (ângulos das 6 juntas, em radianos) ----------
# Repouso = pose "home" do Menagerie: braço erguido, antebraço na horizontal, flange para baixo
REPOUSO = [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0]
# Flange ~6 cm acima do centro da PCB, apontando para baixo
SOBRE_PCB = [0.0, 0.96, 1.00, 0.0, 0.04, 0.0]

# ---------- Faixa de varredura de cada junta (graus) ----------
# Juntas 1, 4 e 6: limite real completo (±360°).
# Juntas 2, 3 e 5: faixa reduzida. No limite real o braço bate nele mesmo ou na bancada.
#   junta  limite real       faixa usada
#   2      -150° a +150°     -130° a  +75°
#   3        -3° a +300°      +10° a +285°
#   5      -124° a +124°      -90° a  +90°
FAIXA = [
    (-360, 360),  # junta 1 - base
    (-130, 75),   # junta 2 - ombro
    (10, 285),    # junta 3 - cotovelo
    (-360, 360),  # junta 4 - giro do antebraço
    (-90, 90),    # junta 5 - punho
    (-360, 360),  # junta 6 - flange
]

VELOCIDADE = 1.5   # rad/s - velocidade das varreduras
DT = 0.01          # intervalo entre quadros (s)


def aplicar(q):
    """Coloca o robô na posição q e avança um passo da simulação."""
    data.qpos[:] = q   # posição das juntas
    data.ctrl[:] = q   # alvo dos motores (o Lite 6 do Menagerie tem servos de posição)
    mujoco.mj_step(model, data)


def mover(viewer, q_inicio, q_fim, velocidade=VELOCIDADE):
    """Interpola de q_inicio até q_fim, mexendo todas as juntas ao mesmo tempo."""
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


# Começa na pose de repouso
data.qpos[:] = REPOUSO
data.ctrl[:] = REPOUSO
mujoco.mj_forward(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    esperar(viewer, REPOUSO, 2.0)

    # ---------- 2. Cada junta individualmente ----------
    for j in range(model.njnt):
        nome = model.joint(j).name
        lim_min, lim_max = model.jnt_range[j]
        minimo = math.radians(FAIXA[j][0])
        maximo = math.radians(FAIXA[j][1])
        print(f"{nome}: limite real {math.degrees(lim_min):.0f}° a {math.degrees(lim_max):.0f}° | "
              f"varrendo {FAIXA[j][0]}° a {FAIXA[j][1]}°")

        q_min = list(REPOUSO)
        q_min[j] = minimo
        q_max = list(REPOUSO)
        q_max[j] = maximo

        mover(viewer, REPOUSO, q_min)
        mover(viewer, q_min, q_max)
        mover(viewer, q_max, REPOUSO)
        esperar(viewer, REPOUSO, 0.5)

    # ---------- 3. Movimento coordenado até a PCB ----------
    print("Movimento coordenado: repouso -> acima da PCB")
    mover(viewer, REPOUSO, SOBRE_PCB, velocidade=0.5)  # mais devagar para ficar visível
    esperar(viewer, SOBRE_PCB, 2.0)
    print("Voltando ao repouso")
    mover(viewer, SOBRE_PCB, REPOUSO, velocidade=0.5)

    # Deixa a janela aberta até o usuário fechar
    print("Fim. Feche a janela para sair.")
    while viewer.is_running():
        esperar(viewer, REPOUSO, 0.1)