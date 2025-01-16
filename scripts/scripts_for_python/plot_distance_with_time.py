#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

# ============================================================================
# CONFIGURAÇÕES INICIAIS
# ============================================================================
# Arquivos e nomes de robô
bag_files = [
    ("multi_tec1.bag", "migbot_kel"),
    ("multi_tec2.bag", "migbot_lyu"),
    ("multi_tec3.bag", "migbot_apf")
]

# Obstáculos de interesse (nomes exatos do /scenario/output_obstacles)
obstacles_original_names = [
    "vegetation3_buoy",
    "vegetation3_buoy_clone",
    "vegetation3_buoy_clone_clone",
    "vegetation3_buoy_clone_clone_clone",
    "branche3_buoy",
    "trunk1_buoy"
]

# Mapeia nomes originais para etiquetas mais simples (obst1, obst2, ..., obst6)
obs_name_map = {
    original: f"obst{i+1}" for i, original in enumerate(obstacles_original_names)
}

# Raio máximo para considerar (filtro < 41.18)
DIST_MAX_PLOT = 41.18

# Linhas horizontais de referência
lines_to_draw = [
    (11.72, "dm"),
    (1.9,   "colisão obst."),
    (4.55,  "colisão obst6")
]

# Objetivo
goal_position = (70, 70)  # Coordenadas do objetivo
goal_threshold = 5.0      # Distância para considerar que o robô alcançou o objetivo

# ============================================================================
# FUNÇÃO PARA LER UMA .BAG E CALCULAR DISTÂNCIAS
# ============================================================================
def read_bag_and_calc_dist(bag_file):
    """
    Lê:
      - /clock
      - /scenario/output_robot (posição do robô)
      - /scenario/output_obstacles (posições dos obstáculos)

    Retorna:
      - start_time (float): instante em que o robô está mais perto de (0,0)
      - end_time   (float)
      - dist_data: dicionário { obs_label: [(t_sim, dist), ...], ... }
        com dist < DIST_MAX_PLOT
      - t_reach_goal (float): instante em que o robô está a <=5 metros do objetivo
      - total_distance (float): distância total percorrida desde start_time até t_reach_goal
    """
    from collections import defaultdict
    obs_positions = defaultdict(list)  # {obs_label: [(t_sim, x, y), ...], ...}
    robot_positions = []               # [(t_sim, x, y), ...]
    current_sim_time = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(
            topics=['/clock', '/scenario/output_robot', '/scenario/output_obstacles']
        ):
            if topic == '/clock':
                current_sim_time = msg.clock.to_sec()

            elif topic == '/scenario/output_robot':
                if current_sim_time is None:
                    continue
                # posição do robô
                rx = msg.position.x
                ry = msg.position.y
                robot_positions.append((current_sim_time, rx, ry))

            elif topic == '/scenario/output_obstacles':
                if current_sim_time is None:
                    continue
                # cada mensagem tem um array de obstáculos
                for obs in msg.obstacles:
                    obs_name = obs.name
                    if obs_name in obs_name_map:  # só interessa se está na lista
                        ox = obs.position.x
                        oy = obs.position.y
                        obs_label = obs_name_map[obs_name]
                        obs_positions[obs_label].append((current_sim_time, ox, oy))

    # Ordenar
    robot_positions.sort(key=lambda x: x[0])
    for k in obs_positions:
        obs_positions[k].sort(key=lambda x: x[0])

    if not robot_positions:
        raise RuntimeError(f"Nenhuma posição do robô encontrada em {bag_file}.")

    # Determinar start_time (quando o robô está mais próximo de (0,0))
    min_dist = float('inf')
    best_time = None
    for (t_sim, rx, ry) in robot_positions:
        d0 = np.hypot(rx, ry)
        if d0 < min_dist:
            min_dist = d0
            best_time = t_sim
    start_time = best_time

    # Determinar t_reach_goal (primeiro instante após start_time com dist <=5)
    t_reach_goal = None
    for (t_sim, rx, ry) in robot_positions:
        if t_sim < start_time:
            continue
        d_goal = np.hypot(rx - goal_position[0], ry - goal_position[1])
        if d_goal <= goal_threshold:
            t_reach_goal = t_sim
            break

    if t_reach_goal is None:
        print(f"  [AVISO] Robô '{bag_file}' não alcançou o objetivo (distância <= {goal_threshold} m).")
        # Definir t_reach_goal como end_time para calcular a distância total até o final
        t_reach_goal = robot_positions[-1][0]

    # Determinar end_time
    end_time = t_reach_goal

    # Calcular distância total percorrida desde start_time até t_reach_goal
    # Somar as distâncias entre posições consecutivas dentro desse intervalo
    total_distance = 0.0
    previous_pos = None
    for (t_sim, rx, ry) in robot_positions:
        if t_sim < start_time:
            continue
        if t_sim > t_reach_goal:
            break
        current_pos = (rx, ry)
        if previous_pos is not None:
            step_dist = np.hypot(current_pos[0] - previous_pos[0], current_pos[1] - previous_pos[1])
            total_distance += step_dist
        previous_pos = current_pos

    # Agora, calcular a distância robô->obst para cada obs e cada timestamp
    # Precisamos "sincronizar" as posições do robô e do obst em cada t.
    #
    # Estratégia simples:
    #   - Para cada obs_label
    #       - Percorrer cada entrada (t_obs, ox, oy)
    #       - Achar a posição do robô no "timestamp" mais próximo (ou exato).
    #         (ex: usar busca binária, ou percorrer. Simples aqui é percorrer ou np.searchsorted)
    #       - Calcular dist, se < DIST_MAX_PLOT, guardar (t_obs, dist)
    #
    # dist_data[obs_label] = [(t, dist), ...]
    #
    # Obs: Se as freq. de obs e robô forem altas, esse loop pode ser caro,
    # mas normalmente o .bag não deve ser tão grande ou podemos filtrar.

    dist_data = { name: [] for name in obs_name_map.values() }  # obst1..6

    # Cria arrays do robô para facilitar
    rob_times = np.array([r[0] for r in robot_positions])
    rob_xs    = np.array([r[1] for r in robot_positions])
    rob_ys    = np.array([r[2] for r in robot_positions])

    # Função que acha (rx, ry) no tempo "t_obs"
    #   -> iremos interpolar ou achar o ponto mais próximo
    #   -> para simplificar, vamos achar o ponto de tempo "não maior" que t_obs
    #      ou o exato se existir
    for obs_label, points in obs_positions.items():
        # obs_label ex: "obst1"
        for (t_obs, ox, oy) in points:
            # Achar índice no rob_times tal que rob_times[idx] <= t_obs < rob_times[idx+1]
            # se t_obs < rob_times[0], não calculamos
            if t_obs < rob_times[0]:
                continue
            # se t_obs > rob_times[-1], a pos do robô + obs não sincroniza
            if t_obs > rob_times[-1]:
                break
            # Agora achamos idx = np.searchsorted(rob_times, t_obs)
            idx = np.searchsorted(rob_times, t_obs)
            if idx == len(rob_times):
                idx = len(rob_times)-1
            # Verifica se passamos
            if rob_times[idx] > t_obs and idx > 0:
                # vamos usar idx-1?
                if abs(rob_times[idx] - t_obs) > abs(rob_times[idx-1] - t_obs):
                    idx = idx-1
            # Pega pos do robô
            rx = rob_xs[idx]
            ry = rob_ys[idx]
            # Calcula dist
            dd = np.hypot(ox - rx, oy - ry)
            if dd <= DIST_MAX_PLOT:
                dist_data[obs_label].append((t_obs, dd))

    # Ordenar dist_data por tempo
    for obs_label in dist_data:
        dist_data[obs_label].sort(key=lambda x: x[0])

    return start_time, t_reach_goal, total_distance, dist_data

# ============================================================================
# FUNÇÃO PARA CRIAR SUBPLOTS
# ============================================================================
def plot_all_bags_with_metrics(bag_files, output_filename="dist_obst_all.eps"):
    """
    Cria uma figura com 3 subplots, um para cada bag.
    Salva a figura como um único arquivo EPS contendo todos os gráficos.
    Além disso, imprime o tempo para alcançar o objetivo e a distância total percorrida.
    """
    # Configurações da figura
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 18), sharex=False)
    fig.subplots_adjust(hspace=0.4)  # Espaço entre subplots

    # Cores para obstáculos
    colors = ['blue', 'green', 'red', 'purple', 'brown', 'orange']

    for idx, (bag_file, robot_name) in enumerate(bag_files):
        ax = axes[idx]
        print(f"\nLendo {bag_file} (robô '{robot_name}') ...")
        start_time, t_reach_goal, total_distance, dist_data = read_bag_and_calc_dist(bag_file)
        time_to_goal = t_reach_goal - start_time
        print(f"  - start_time = {start_time:.2f}, t_reach_goal = {t_reach_goal:.2f}")
        print(f"  - Tempo para alcançar o objetivo: {time_to_goal:.2f} segundos")
        print(f"  - Distância total percorrida: {total_distance:.2f} metros")

        # 'dist_data' é { "obst1": [(t_sim, dist), ...], "obst2": [...], ... }
        # Precisamos montar o plot para os 6 obstáculos
        # Eixo X: t - start_time
        # Eixo Y: dist
        # Se um obstáculo não tiver dados, não plota

        plotted_any = False
        for i, (obs_label, points) in enumerate(sorted(dist_data.items())):
            if not points:
                continue
            t_vals = [t - start_time for (t, d) in points if t >= start_time]
            d_vals = [d for (t, d) in points if t >= start_time]
            if not t_vals:
                continue
            plotted_any = True
            color = colors[i % len(colors)]
            ax.plot(t_vals, d_vals, label=obs_label, color=color, linestyle='-')

            # Adicionar marcador no último ponto
            ax.plot(t_vals[-1], d_vals[-1], marker='o', color=color)

        if not plotted_any:
            ax.text(0.5, 0.5, "Nenhuma distância < 41.18 encontrada.",
                    horizontalalignment='center',
                    verticalalignment='center',
                    transform=ax.transAxes,
                    fontsize=12, color='red')
            print("  [AVISO] Nenhuma distância < 41.18 encontrada após start_time.")
        else:
            # Linhas horizontais
            for (y_line, label_line) in lines_to_draw:
                ax.axhline(y=y_line, color='gray', linestyle='--', alpha=0.7)
                # Ajustar posição do texto
                ax.text(ax.get_xlim()[0], y_line + 0.2, label_line,
                        color='gray', fontsize=10, verticalalignment='bottom')

            ax.set_title(f"{robot_name} - Distância aos Obstáculos vs Tempo")
            ax.set_xlabel("Tempo (s) desde start_time")
            ax.set_ylabel("Distância (m) até obstáculos")
            ax.legend()
            ax.grid(True)

            # Informações adicionais no gráfico
            ax.text(0.95, 0.95, f"Tempo para objetivo: {time_to_goal:.2f}s\n"
                                 f"Distância total: {total_distance:.2f}m",
                    transform=ax.transAxes,
                    fontsize=10,
                    verticalalignment='top',
                    horizontalalignment='right',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.5))

        print(f"  -> Subplot para '{robot_name}' adicionado.")

    # Salvar a figura com todos os subplots
    plt.tight_layout()
    plt.savefig(output_filename, format='eps')
    print(f"\nTodos os gráficos foram salvos em '{output_filename}'.")

    # Exibir a figura
    plt.show()

    # Opcional: Salvar as métricas em um arquivo de texto
    with open("metrics_distance.txt", "w") as f:
        for (bag_file, robot_name) in bag_files:
            try:
                _, t_reach_goal, total_distance, _ = read_bag_and_calc_dist(bag_file)
                time_to_goal = t_reach_goal - read_bag_and_calc_dist(bag_file)[0]
                f.write(f"{robot_name}:\n")
                f.write(f"  Tempo para alcançar o objetivo: {time_to_goal:.2f} segundos\n")
                f.write(f"  Distância total percorrida: {total_distance:.2f} metros\n\n")
            except Exception as e:
                f.write(f"{robot_name}:\n")
                f.write(f"  Erro: {e}\n\n")

    print("\nConcluído. Foram gerados gráficos e métricas salvos em 'metrics_distance.txt'.")

# ============================================================================
# MAIN
# ============================================================================
def main():
    # Nome do arquivo de saída
    output_eps = "dist_obst_all.eps"

    # Criar e salvar os subplots com métricas
    plot_all_bags_with_metrics(bag_files, output_filename=output_eps)

# ============================================================================
if __name__ == "__main__":
    main()
