#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import euler_from_quaternion
from bisect import bisect_left
import sys

# ============================================================================
# CONFIGURAÇÕES INICIAIS
# ============================================================================

# Defina PLOT_MODE para escolher o gráfico desejado:
#  1 => Plot do Rastro (robô + obstáculos)
#  2 => Plot das Forças (magnitudes normalizadas, excluindo total_force)
#  3 => Plot da Distância até o Obstáculo
#  4 => Plot do Heading e Velocidade (normalizada)
PLOT_MODE = 2  # Altere para 2, 3 ou 4 conforme desejar

# Caminho para o arquivo .bag
bag_file = "dissertacao_tec1.bag"

# Tópicos de interesse
robot_topic = "/scenario/output_robot"
obstacles_topic = "/scenario/output_obstacles"
force_topics = [
    "/apfm/attractive_force",
    "/apfm/dynamic_force",
    "/apfm/emergency_force",
    "/apfm/static_force",
    # "/apfm/total_force",  # Excluído (não plotar força total)
]

# Número total de cenários
NUM_CENARIOS = 21

# Limiares para detecção de (0,0) e (70,70)
DIST_THRESHOLD_START = 3.0
DIST_THRESHOLD_GOAL  = 5.0

# Formato de saída dos gráficos
EPS_FORMAT = "eps"

# Diâmetros dos objetos (em metros)
DIAMETERS = {
    "vegetation3_buoy": 1.0,
    "trunk1_buoy": 6.3,
    "migbot": 2.8
}

# ============================================================================
# FUNÇÕES AUXILIARES
# ============================================================================

def quaternion_to_yaw_degrees(qx, qy, qz, qw):
    """
    Converte quaternion em yaw (graus).
    """
    euler_rad = euler_from_quaternion([qx, qy, qz, qw])
    yaw_deg = np.degrees(euler_rad[2])
    return yaw_deg

def magnitude2D(vx, vy):
    """
    Calcula a magnitude de um vetor 2D.
    """
    return np.hypot(vx, vy)

def is_in_domain(x, y):
    """
    Retorna True se (x,y) estiver dentro do quadrante [0,70] x [0,70].
    """
    return (0 <= x <= 70) and (0 <= y <= 70)

def find_closest_robot_idx(robot_times, t):
    """
    Encontra o índice do robô com timestamp mais próximo de t usando busca binária.
    """
    pos = bisect_left(robot_times, t)
    if pos == 0:
        return 0
    if pos == len(robot_times):
        return len(robot_times) - 1
    before = pos - 1
    after = pos
    if abs(robot_times[after] - t) < abs(robot_times[before] - t):
        return after
    else:
        return before

def find_exit_point(x1, y1, x2, y2):
    """
    Encontra o ponto onde a reta de (x1, y1) para (x2, y2) sai do domínio [0,70] x [0,70].
    Retorna as coordenadas (x_exit, y_exit). Se não houver saída, retorna None.
    """
    # Define as bordas do domínio
    boundaries = [
        ((0, 0), (0, 70)),     # Esquerda
        ((0, 70), (70, 70)),   # Cima
        ((70, 70), (70, 0)),   # Direita
        ((70, 0), (0, 0))      # Baixo
    ]
    
    # Função para verificar interseção entre duas retas
    def lines_intersection(p1, p2, p3, p4):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
        if denom == 0:
            return None  # Retas paralelas

        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom

        return (px, py)

    # Iterar sobre todas as bordas para encontrar a interseção
    for boundary in boundaries:
        p3, p4 = boundary
        intersect = lines_intersection((x1, y1), (x2, y2), p3, p4)
        if intersect:
            px, py = intersect
            # Verificar se o ponto está entre (x1, y1) e (x2, y2)
            if min(x1, x2) - 1e-6 <= px <= max(x1, x2) + 1e-6 and min(y1, y2) - 1e-6 <= py <= max(y1, y2) + 1e-6:
                # Verificar se o ponto está dentro do domínio
                if 0 <= px <= 70 and 0 <= py <= 70:
                    return (px, py)
    return None

# ============================================================================
# 1) Plot do Rastro (PLOT_MODE=1)
# ============================================================================

def plot_rastro(data_robot, data_obstacle, scenario_id):
    """
    Gera o gráfico de rastro do robô e obstáculos.
    Adiciona círculos representando os diâmetros e pontos de saída do domínio.

    Args:
        data_robot (list): Lista de tuplas (t, x, y, vx, vy, yaw_deg).
        data_obstacle (dict): Dict { obst_name: list of (t, x, y, vx, vy) }.
        scenario_id (int): ID do cenário para nomeação do arquivo.
    """
    if not data_robot:
        print("[AVISO] Sem dados do robô para plotar rastro.")
        return

    # Extrair rastro do robô
    xs_robot = [r[1] for r in data_robot]  # x
    ys_robot = [r[2] for r in data_robot]  # y

    plt.figure(figsize=(10,8))
    plt.plot(xs_robot, ys_robot, label="Robô", color='blue')

    # Adicionar círculo para o robô (última posição)
    robot_last = data_robot[-1]
    robot_circle = plt.Circle((robot_last[1], robot_last[2]), DIAMETERS["migbot"]/2, color='blue', fill=False, linewidth=2, label="Migbot")
    plt.gca().add_patch(robot_circle)

    # Limites do domínio
    plt.xlim(0, 70)
    plt.ylim(0, 70)

    # Desenhar as bordas do domínio
    plt.plot([0, 70, 70, 0, 0], [0, 0, 70, 70, 0], 'k-', linewidth=1)

    # Adicionar obstáculos
    exit_points = []
    for obst_name, obst_list in data_obstacle.items():
        if not obst_list:
            continue
        xs_obst = [o[1] for o in obst_list]  # x
        ys_obst = [o[2] for o in obst_list]  # y

        # Plotar trajeto do obstáculo
        plt.plot(xs_obst, ys_obst, label=obst_name, linestyle='--', linewidth=2)

        # Verificar se o último ponto está dentro do domínio
        obst_last = obst_list[-1]
        if is_in_domain(obst_last[1], obst_last[2]):
            # Adicionar círculo na última posição dentro do domínio
            obst_circle = plt.Circle((obst_last[1], obst_last[2]), DIAMETERS[obst_name]/2, color='red', fill=False, linewidth=2, label=obst_name)
            plt.gca().add_patch(obst_circle)
        else:
            # Encontrar ponto de saída
            if len(obst_list) >= 2:
                # Iterar de trás para frente para encontrar o último ponto dentro do domínio
                for i in range(len(obst_list)-1, -1, -1):
                    x_prev, y_prev = obst_list[i][1], obst_list[i][2]
                    if is_in_domain(x_prev, y_prev):
                        break
                else:
                    # Nenhum ponto dentro do domínio
                    x_prev, y_prev = None, None

                if x_prev is not None and y_prev is not None:
                    # Encontrar ponto de saída entre (x_prev, y_prev) e (x_curr, y_curr)
                    x_curr, y_curr = obst_last[1], obst_last[2]
                    exit_point = find_exit_point(x_prev, y_prev, x_curr, y_curr)
                    if exit_point:
                        exit_points.append((exit_point, obst_name))
                        # Adicionar círculo no ponto de saída
                        plt.plot(exit_point[0], exit_point[1], 'ro', markersize=8)
                    
                    # Adicionar círculo no último ponto dentro do domínio
                    plt.plot(x_prev, y_prev, 'mo', markersize=8, label=f"{obst_name} Último Dentro")

    plt.title(f"Cenário {scenario_id}: Rastro (Robô + Obstáculos)")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"plot_rastro_scenario{scenario_id}.{EPS_FORMAT}", format=EPS_FORMAT)
    plt.show()

# ============================================================================
# 2) Plot das Forças Normalizadas (PLOT_MODE=2)
# ============================================================================

def plot_forcas(data_forces, data_robot, scenario_id):
    """
    Gera o gráfico das forças (magnitudes), excluindo total_force.
    Normaliza as forças com base na força máxima encontrada.

    Args:
        data_forces (dict): Dict { force_topic: list of (t, fx, fy) }.
        data_robot (list): Lista de tuplas (t, x, y, vx, vy, yaw_deg).
        scenario_id (int): ID do cenário para nomeação do arquivo.
    """
    if not data_forces:
        print("[AVISO] Sem dados de forças.")
        return

    plt.figure(figsize=(10,8))
    max_force = 0.0

    # Primeiro, encontre a força máxima para normalização
    for ftopic, flist in data_forces.items():
        for f in flist:
            mag = magnitude2D(f[1], f[2])
            if mag > max_force:
                max_force = mag

    # Evitar divisão por zero
    if max_force == 0:
        max_force = 1.0

    # Plotar cada força normalizada
    for ftopic, flist in data_forces.items():
        if not flist:
            continue
        times = [f[0] for f in flist]
        mags = [magnitude2D(f[1], f[2])/max_force for f in flist]
        label_ = ftopic.split('/')[-1]  # Ex: "attractive_force"
        plt.plot(times, mags, label=label_)

    plt.title(f"Cenário {scenario_id}: Forças (Magnitudes Normalizadas)")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Magnitude da Força (Normalizada 0..1)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"plot_forcas_scenario{scenario_id}.{EPS_FORMAT}", format=EPS_FORMAT)
    plt.show()

# ============================================================================
# 3) Plot da Distância (PLOT_MODE=3)
# ============================================================================

def plot_dist_obst(data_robot, data_obstacle, robot_times, scenario_id):
    """
    Gera gráfico da distância do robô ao obstáculo ao longo do tempo.

    Args:
        data_robot (list): Lista de tuplas (t, x, y, vx, vy, yaw_deg).
        data_obstacle (dict): Dict { obst_name: list of (t, x, y, vx, vy) }.
        robot_times (list): Lista de timestamps do robô para busca binária.
        scenario_id (int): ID do cenário para nomeação do arquivo.
    """
    if (not data_robot) or (not data_obstacle):
        print("[AVISO] Sem dados suficientes para dist obst.")
        return

    plt.figure(figsize=(10,8))

    for obst_name, obst_list in data_obstacle.items():
        if not obst_list:
            continue
        # Inicializar listas de tempo e distância
        dist_times = []
        dist_vals = []

        for (t_o, ox, oy, _, _) in obst_list:
            # Encontrar índice do robô com timestamp mais próximo
            ridx = find_closest_robot_idx(robot_times, t_o)
            if ridx < len(data_robot):
                rx, ry = data_robot[ridx][1], data_robot[ridx][2]
                dist = magnitude2D(rx - ox, ry - oy)
                dist_times.append(t_o)
                dist_vals.append(dist)

        if dist_times:
            plt.plot(dist_times, dist_vals, label=f"Distância a {obst_name}")

    plt.title(f"Cenário {scenario_id}: Distância ao Obstáculo ao Longo do Tempo")
    plt.xlabel("Tempo (s)")
    plt.ylabel("Distância (m)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"plot_dist_obst_scenario{scenario_id}.{EPS_FORMAT}", format=EPS_FORMAT)
    plt.show()

# ============================================================================
# 4) Plot do Heading e Velocidade Normalizada (PLOT_MODE=4)
# ============================================================================

def plot_heading_vel(data_robot, scenario_id):
    """
    Gera gráfico de heading e velocidade (normalizada).

    Args:
        data_robot (list): Lista de tuplas (t, x, y, vx, vy, yaw_deg).
        scenario_id (int): ID do cenário para nomeação do arquivo.
    """
    if not data_robot:
        print("[AVISO] Sem dados do robô.")
        return

    times = [r[0] for r in data_robot]
    yaws = [r[5] for r in data_robot]
    speeds = [magnitude2D(r[3], r[4]) for r in data_robot]
    max_speed = max(speeds) if speeds else 1.0
    norm_speeds = [s / max_speed if max_speed > 0 else 0.0 for s in speeds]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Gráfico de Heading
    ax1.plot(times, yaws, label="Heading (°)", color='blue')
    ax1.set_title(f"Cenário {scenario_id}: Heading ao Longo do Tempo")
    ax1.set_xlabel("Tempo (s)")
    ax1.set_ylabel("Heading (°)")
    ax1.legend()
    ax1.grid(True)

    # Gráfico de Velocidade Normalizada
    ax2.plot(times, norm_speeds, label="Velocidade Normalizada", color='green')
    ax2.set_title(f"Cenário {scenario_id}: Velocidade Normalizada ao Longo do Tempo")
    ax2.set_xlabel("Tempo (s)")
    ax2.set_ylabel("Velocidade Normalizada (0..1)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig(f"plot_heading_vel_scenario{scenario_id}.{EPS_FORMAT}", format=EPS_FORMAT)
    plt.show()

# ============================================================================
# PROCESSAR CADA CENÁRIO
# ============================================================================

def process_scenario_data(scenario_id, data_robot, data_obstacle, data_forces, robot_times):
    """
    Processa e gera os gráficos para um único cenário.
    
    Args:
        scenario_id (int): ID do cenário (1 a 21).
        data_robot (list): Lista de tuplas (t, x, y, vx, vy, yaw_deg).
        data_obstacle (dict): Dict { obst_name: list of (t, x, y, vx, vy) }.
        data_forces (dict): Dict { force_topic: list of (t, fx, fy) }.
        robot_times (list): Lista de timestamps do robô para busca binária.
    """
    if not data_robot:
        print(f"[AVISO] Cenário {scenario_id}: sem dados de robô.")
        return

    # Calcular distância percorrida
    xs = [r[1] for r in data_robot]
    ys = [r[2] for r in data_robot]
    dist_total = np.sum([magnitude2D(xs[i] - xs[i-1], ys[i] - ys[i-1]) for i in range(1, len(xs))])

    # Tempo total do cenário
    time_total = data_robot[-1][0] - data_robot[0][0] if data_robot else 0.0

    # Gerar gráficos conforme PLOT_MODE
    if PLOT_MODE == 1:
        # Plot Rastro
        plot_rastro(data_robot, data_obstacle, scenario_id)

    elif PLOT_MODE == 2:
        # Plot Forças Normalizadas (excluindo total_force)
        plot_forcas(data_forces, data_robot, scenario_id)

    elif PLOT_MODE == 3:
        # Plot Distância ao Obstáculo
        plot_dist_obst(data_robot, data_obstacle, robot_times, scenario_id)

    elif PLOT_MODE == 4:
        # Plot Heading e Velocidade
        plot_heading_vel(data_robot, scenario_id)

    else:
        print(f"[AVISO] Valor de PLOT_MODE={PLOT_MODE} inválido. Use 1..4.")
        return

    # Imprimir Métricas
    print(f"Cenário {scenario_id}:")
    print(f"  Tempo Total: {time_total:.2f} segundos")
    print(f"  Distância Percorrida: {dist_total:.2f} metros\n")

# ============================================================================
# FUNÇÃO PRINCIPAL
# ============================================================================

def main():
    print("=== Iniciando Processamento do Arquivo 'dissertacao_tec1.bag' ===")

    # 1) Ler integralmente o .bag e salvar em variáveis
    print("Lendo dados do bag...")
    all_robot_data = []          # Lista de tuplas: (t, x, y, vx, vy, yaw_deg)
    all_obstacles_data = {}      # Dict: obst_name -> lista de tuplas (t, x, y, vx, vy)
    all_forces_data = {}         # Dict: force_topic -> lista de tuplas (t, fx, fy)

    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[robot_topic, obstacles_topic] + force_topics):
                tsec = t.to_sec()

                if topic == robot_topic:
                    # Extrair dados do robô
                    rx = msg.position.x
                    ry = msg.position.y
                    vx = msg.velocity.x
                    vy = msg.velocity.y
                    qx = msg.orientation.x
                    qy = msg.orientation.y
                    qz = msg.orientation.z
                    qw = msg.orientation.w
                    yaw_deg = quaternion_to_yaw_degrees(qx, qy, qz, qw)
                    all_robot_data.append((tsec, rx, ry, vx, vy, yaw_deg))

                elif topic == obstacles_topic:
                    # Extrair dados dos obstáculos com nomes específicos
                    for obs in msg.obstacles:
                        obst_name = obs.name
                        if obst_name not in ["vegetation3_buoy", "trunk1_buoy"]:
                            continue  # Ignora outros obstáculos
                        ox = obs.position.x
                        oy = obs.position.y
                        ovx = obs.velocity.x
                        ovy = obs.velocity.y
                        if obst_name not in all_obstacles_data:
                            all_obstacles_data[obst_name] = []
                        all_obstacles_data[obst_name].append((tsec, ox, oy, ovx, ovy))

                elif topic in force_topics:
                    # Extrair dados das forças
                    fx = msg.x
                    fy = msg.y
                    if topic not in all_forces_data:
                        all_forces_data[topic] = []
                    all_forces_data[topic].append((tsec, fx, fy))
    except FileNotFoundError:
        print(f"[ERRO] Arquivo '{bag_file}' não encontrado. Verifique o caminho.")
        sys.exit(1)
    except Exception as e:
        print(f"[ERRO] Ocorreu um erro ao ler o bag: {e}")
        sys.exit(1)

    print("Leitura do bag concluída.")
    print(f"  - Dados do Robô: {len(all_robot_data)} registros.")
    print(f"  - Obstáculos: {len(all_obstacles_data)} tipos.")
    print(f"  - Forças: {len(all_forces_data)} tipos.\n")

    # Ordenar dados do robô por tempo (se não estiver ordenado)
    all_robot_data.sort(key=lambda r: r[0])
    robot_times = [r[0] for r in all_robot_data]

    # 2) Segmentar em 21 cenários consecutivos
    print("Segmentando dados em 21 cenários consecutivos...")
    scenario_blocks = []  # Lista de tuplas: (start_idx, end_idx)
    in_scenario = False
    current_start = 0
    found_count = 0

    for i, row in enumerate(all_robot_data):
        t, x, y, vx, vy, yaw = row
        dist_to_start = np.hypot(x, y)  # Distância ao ponto (0,0)
        dist_to_goal = np.hypot(x - 70, y - 70)  # Distância ao ponto (70,70)

        if not in_scenario and dist_to_start <= DIST_THRESHOLD_START:
            # Início de um novo cenário
            in_scenario = True
            current_start = i
            print(f"  - Início do Cenário {found_count + 1} em índice {current_start} (t={t:.2f}s)")

        if in_scenario and dist_to_goal <= DIST_THRESHOLD_GOAL:
            # Fim do cenário
            scenario_blocks.append((current_start, i))
            found_count += 1
            print(f"  - Fim do Cenário {found_count} em índice {i} (t={t:.2f}s)")
            in_scenario = False

            if found_count == NUM_CENARIOS:
                break

    if found_count < NUM_CENARIOS:
        print(f"\n[AVISO] Apenas {found_count} cenários foram detectados (esperava {NUM_CENARIOS}).")
    else:
        print(f"\nTodos os {NUM_CENARIOS} cenários foram detectados com sucesso.\n")

    # 3) Processar cada cenário
    for scenario_id, (start_idx, end_idx) in enumerate(scenario_blocks, start=1):
        print(f"=== Processando Cenário {scenario_id} ===")

        # Extrair dados do robô para o cenário
        data_robot_s = all_robot_data[start_idx:end_idx+1]

        # Filtrar dados dos obstáculos para o cenário
        data_obstacle_s = {}
        for obst_name, obst_list in all_obstacles_data.items():
            sublist = []
            for ob in obst_list:
                t_obst = ob[0]
                closest_idx = find_closest_robot_idx(robot_times, t_obst)
                if start_idx <= closest_idx <= end_idx:
                    sublist.append(ob)
            data_obstacle_s[obst_name] = sublist

        # Filtrar dados das forças para o cenário
        data_forces_s = {}
        for ftopic, flist in all_forces_data.items():
            subf = []
            for f in flist:
                t_force = f[0]
                closest_idx = find_closest_robot_idx(robot_times, t_force)
                if start_idx <= closest_idx <= end_idx:
                    subf.append(f)
            data_forces_s[ftopic] = subf

        # Gerar e exibir os gráficos conforme PLOT_MODE
        process_scenario_data(scenario_id, data_robot_s, data_obstacle_s, data_forces_s, robot_times[start_idx:end_idx+1])

    print("=== Processamento dos 21 cenários concluído ===")
    print("Todos os gráficos foram gerados e salvos. Feche cada gráfico para exibir o próximo.\n")
    print("Fim do script.")

# ============================================================================
# EXECUÇÃO DO SCRIPT
# ============================================================================

if __name__ == "__main__":
    main()
