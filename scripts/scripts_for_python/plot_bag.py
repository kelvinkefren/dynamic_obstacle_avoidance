#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
from collections import defaultdict
import numpy as np
import os

# -------------------------------------------------------
# Configurações Gerais
# -------------------------------------------------------

# Nomes dos arquivos .bag
bag_file_1 = 'multi_tec1.bag'
bag_file_2 = 'multi_tec2.bag'
bag_file_3 = 'multi_tec3.bag'

# Obstáculos existem apenas no multi_tec1 (iguais nos 3)
obstacles_original_names = [
    'vegetation3_buoy',
    'vegetation3_buoy_clone',
    'vegetation3_buoy_clone_clone',
    'vegetation3_buoy_clone_clone_clone',
    'branche3_buoy',
    'trunk1_buoy'
]
obstacle_name_mapping = {
    orig: f'obst{i+1}'
    for i, orig in enumerate(obstacles_original_names)
}

# Diâmetros fixos
obstacle_diameters = {
    'obst1': 1.0,
    'obst2': 1.0,
    'obst3': 1.0,
    'obst4': 1.0,
    'obst5': 1.0,
    'obst6': 6.3
}
robot_diameter = 2.8  # Robô (independente da bag)

# Definições dos nomes de cada robô
# multi_tec1 -> migbot_kel
# multi_tec2 -> migbot_lyu
# multi_tec3 -> migbot_apf
robot_labels = {
    bag_file_1: "migbot_kel",
    bag_file_2: "migbot_lyu",
    bag_file_3: "migbot_apf"
}

# Tamanho e posição do objetivo
goal_position = (70, 70)
goal_size = 2.0

# Tempos de plotagem (segundos após o start_time de cada robô)
plot_seconds = [5, 10, 15, 23, 30, 45]

# -------------------------------------------------------
# Função auxiliar: desenhar o triângulo do objetivo
# -------------------------------------------------------
def draw_goal(ax, position, size, label="Goal"):
    x, y = position
    triangle = Polygon([
        (x, y + size),      # Topo
        (x - size / 2, y - size / 2),   # Inferior esquerdo
        (x + size / 2, y - size / 2)    # Inferior direito
    ], closed=True, color='red', alpha=0.7)
    ax.add_patch(triangle)

    # Texto
    ax.text(x, y - size - 0.5, label,
            color='black', fontsize=12,
            ha='center', va='top', weight='bold')

# -------------------------------------------------------
# Função para ler APENAS robô de uma .bag
# ou obstáculos + robô se for o multi_tec1
# -------------------------------------------------------
def read_bag_data(bag_file, is_obstacle_bag=False):
    """
    Lê o arquivo .bag e retorna:
      - obstacle_data (dicionário) se is_obstacle_bag=True, caso contrário vazio
      - robot_data (lista)
      - start_time e end_time (float)
    """
    import rosbag

    obstacle_data = defaultdict(list) if is_obstacle_bag else {}
    robot_data = []

    current_sim_time = None
    start_time = None
    end_time = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(
            topics=['/clock', '/scenario/output_obstacles', '/scenario/output_robot']
        ):
            if topic == '/clock':
                current_sim_time = msg.clock.to_sec()

            elif topic == '/scenario/output_obstacles' and is_obstacle_bag:
                if current_sim_time is None:
                    continue
                for obs in msg.obstacles:
                    if obs.name in obstacle_name_mapping:
                        new_name = obstacle_name_mapping[obs.name]
                        obstacle_data[new_name].append(
                            (current_sim_time, obs.position.x, obs.position.y)
                        )

            elif topic == '/scenario/output_robot':
                if current_sim_time is None:
                    continue
                x = msg.position.x
                y = msg.position.y
                robot_data.append((current_sim_time, x, y))

    # Ordenar
    if is_obstacle_bag:
        for obst in obstacle_data:
            obstacle_data[obst].sort(key=lambda x: x[0])

    robot_data.sort(key=lambda x: x[0])

    # Se não tem robô, erro.
    if not robot_data:
        raise RuntimeError(f"Nenhuma posição de robô encontrada em {bag_file}.")

    # Determinar start_time (quando o robô está mais próximo de (0,0))
    min_dist = float('inf')
    best_time = None
    for (t_sim, rx, ry) in robot_data:
        dist = np.hypot(rx, ry)
        if dist < min_dist:
            min_dist = dist
            best_time = t_sim

    start_time = best_time
    # end_time = último tempo do robô ou do obstáculo
    r_end = robot_data[-1][0]
    if is_obstacle_bag and obstacle_data:
        o_end = max((vals[-1][0] for vals in obstacle_data.values() if vals), default=r_end)
        end_time = max(r_end, o_end)
    else:
        end_time = r_end

    return obstacle_data, robot_data, start_time, end_time

# -------------------------------------------------------
# 1) Ler multi_tec1 (obstáculos + robô)
# -------------------------------------------------------
print(f"Lendo {bag_file_1} ... (obstáculos + robô)")
obstacle_data_1, robot_data_1, start_time_1, end_time_1 = read_bag_data(
    bag_file_1, is_obstacle_bag=True
)
print(f"multi_tec1 -> start_time = {start_time_1:.2f}, end_time = {end_time_1:.2f}")

# -------------------------------------------------------
# 2) Ler multi_tec2 (apenas robô)
# -------------------------------------------------------
print(f"Lendo {bag_file_2} ... (somente robô)")
_, robot_data_2, start_time_2, end_time_2 = read_bag_data(
    bag_file_2, is_obstacle_bag=False
)
print(f"multi_tec2 -> start_time = {start_time_2:.2f}, end_time = {end_time_2:.2f}")

# -------------------------------------------------------
# 3) Ler multi_tec3 (apenas robô)
# -------------------------------------------------------
print(f"Lendo {bag_file_3} ... (somente robô)")
_, robot_data_3, start_time_3, end_time_3 = read_bag_data(
    bag_file_3, is_obstacle_bag=False
)
print(f"multi_tec3 -> start_time = {start_time_3:.2f}, end_time = {end_time_3:.2f}")

# -------------------------------------------------------
# Preparar cores e etc. para plotar obstáculos
# -------------------------------------------------------
linewidth_scale = 0.5
cmap = plt.colormaps['tab10']
sorted_obstacles = sorted(obstacle_data_1.keys(),
                          key=lambda x: obstacle_diameters[x], reverse=True)
colors_obstacles = {}
for i, obst_name in enumerate(sorted_obstacles):
    colors_obstacles[obst_name] = cmap(i % 10)

# -------------------------------------------------------
# Preparar uma função auxiliar para filtrar dados até um "plot_time"
# -------------------------------------------------------
def filter_positions(data_list, plot_time):
    """
    data_list = [(t_sim, x, y), ...]
    Retorna xs, ys (somente até plot_time)
    """
    xs = []
    ys = []
    for (t_sim, px, py) in data_list:
        if t_sim <= plot_time:
            xs.append(px)
            ys.append(py)
        else:
            break
    return xs, ys

# -------------------------------------------------------
# Plot e salvamento de cada dt
# -------------------------------------------------------
for dt in plot_seconds:

    # Calcular plot_time (cada bag separadamente)
    # Se exceder end_time, usar end_time
    pt1 = min(start_time_1 + dt, end_time_1)
    pt2 = min(start_time_2 + dt, end_time_2)
    pt3 = min(start_time_3 + dt, end_time_3)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, 80)
    ax.set_ylim(0, 80)
    ax.set_aspect('equal')
    ax.grid(True)

    ax.set_xlabel("Posição X")
    ax.set_ylabel("Posição Y")

    # Desenhar objetivo
    draw_goal(ax, goal_position, goal_size)

    # Plotar obstáculos (do multi_tec1)
    for obst_name in sorted_obstacles:
        obs_data_list = obstacle_data_1[obst_name]
        xs, ys = filter_positions(obs_data_list, pt1)
        if xs and ys:
            lw = obstacle_diameters[obst_name] * linewidth_scale
            ax.plot(xs, ys, label=obst_name, linewidth=lw,
                    color=colors_obstacles[obst_name], alpha=0.8)

            # Círculo na última posição
            lx, ly = xs[-1], ys[-1]
            rr = obstacle_diameters[obst_name] / 2
            circle_obst = patches.Circle((lx, ly), rr,
                                         linewidth=1,
                                         edgecolor=colors_obstacles[obst_name],
                                         facecolor='none',
                                         linestyle='-')
            ax.add_patch(circle_obst)

    # Plotar robô 1 (migbot_kel)
    xs1, ys1 = filter_positions(robot_data_1, pt1)
    if xs1 and ys1:
        lw_r = robot_diameter * linewidth_scale
        ax.plot(xs1, ys1, label="migbot_kel",
                linewidth=lw_r, color='black', linestyle='--')
        # Círculo final
        xlf, ylf = xs1[-1], ys1[-1]
        circle_r1 = patches.Circle((xlf, ylf), robot_diameter/2,
                                   linewidth=1, edgecolor='black',
                                   facecolor='none', linestyle='--')
        ax.add_patch(circle_r1)

    # Plotar robô 2 (migbot_lyu)
    xs2, ys2 = filter_positions(robot_data_2, pt2)
    if xs2 and ys2:
        lw_r = robot_diameter * linewidth_scale
        ax.plot(xs2, ys2, label="migbot_lyu",
                linewidth=lw_r, color='blue', linestyle='-.')
        # Círculo final
        xlf, ylf = xs2[-1], ys2[-1]
        circle_r2 = patches.Circle((xlf, ylf), robot_diameter/2,
                                   linewidth=1, edgecolor='blue',
                                   facecolor='none', linestyle='-.')
        ax.add_patch(circle_r2)

    # Plotar robô 3 (migbot_apf)
    xs3, ys3 = filter_positions(robot_data_3, pt3)
    if xs3 and ys3:
        lw_r = robot_diameter * linewidth_scale
        ax.plot(xs3, ys3, label="migbot_apf",
                linewidth=lw_r, color='green', linestyle=':')
        # Círculo final
        xlf, ylf = xs3[-1], ys3[-1]
        circle_r3 = patches.Circle((xlf, ylf), robot_diameter/2,
                                   linewidth=1, edgecolor='green',
                                   facecolor='none', linestyle=':')
        ax.add_patch(circle_r3)

    ax.legend()
    ax.set_title(f"Rastros até t = {dt:.1f}s (cada robô com seu clock local)")

    # Salvar e mostrar
    output_filename = f"rastro_t{dt}s.eps"
    plt.savefig(output_filename, format='eps')
    print(f"Gráfico salvo como '{output_filename}'")

    plt.show()

print("Finalizado. Todos os gráficos foram gerados e exibidos.")
