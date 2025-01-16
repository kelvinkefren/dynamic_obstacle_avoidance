#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

# ============================================================================
# CONFIGURAÇÕES INICIAIS
# ============================================================================
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

# Mapeia nomes originais para algo mais simples (opcional)
# Ex: veget3_..., veget3_..._clone, etc. → obst1, obst2...
# Pode-se manter o nome original, se preferir.
obs_name_map = {
    original: f"obst{i+1}" for i, original in enumerate(obstacles_original_names)
}

# Raio máximo para considerar (filtro < 41.18)
DIST_MAX_PLOT = 41.18

# Linhas horizontais de referência
lines_to_draw = [
    (14.47, "dm para obst6"),
    (11.72, "dm"),
    (1.9,   "colisão obst."),
    (4.55,  "colisão obst6")
]

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
      - dist_data: dicionário { obs_name: [(t_sim, dist), ...], ... }
        com dist < DIST_MAX_PLOT
    """
    from collections import defaultdict
    obs_positions = defaultdict(list)  # {obs_name: [(t_sim, x, y), ...], ...}
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
                        obs_positions[obs_name].append((current_sim_time, ox, oy))

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

    # end_time = maior timestamp (robô ou obs)
    end_time = start_time
    end_time = max(end_time, robot_positions[-1][0])
    for k in obs_positions:
        if obs_positions[k]:
            end_time = max(end_time, obs_positions[k][-1][0])

    # Agora, calcular a distância robô->obst para cada obs e cada timestamp
    # Precisamos "sincronizar" as posições do robô e do obst em cada t.
    #
    # Estratégia simples:
    #   - Para cada obs_name
    #       - Percorrer cada entrada (t_obs, ox, oy)
    #       - Achar a posição do robô no "timestamp" mais próximo (ou exato).
    #         (ex: usar busca binária, ou percorrer. Simples aqui é percorrer ou np.searchsorted)
    #       - Calcular dist, se < DIST_MAX_PLOT, guardar (t_obs, dist)
    #
    # dist_data[obs_name] = [(t, dist), ...]
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
    for obs_name, points in obs_positions.items():
        # obs_name ex: "vegetation3_buoy"
        obs_label = obs_name_map[obs_name]  # "obst1"...
        for (t_obs, ox, oy) in points:
            # Achar índice no rob_times tal que rob_times[idx] <= t_obs < rob_times[idx+1]
            # se t_obs < rob_times[0], não calculamos
            if t_obs < rob_times[0]:
                continue
            # se t_obs > rob_times[-1], a pos do robô + obs não sincroniza
            if t_obs > rob_times[-1]:
                break
            # Agora achamos idx = np.searchsorted(rob_times, t_obs) ...
            idx = np.searchsorted(rob_times, t_obs)
            if idx == len(rob_times):
                idx = len(rob_times)-1
            # Verifica se passamos
            if rob_times[idx] > t_obs and idx>0:
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

    return start_time, end_time, dist_data

# ============================================================================
# MAIN
# ============================================================================
def main():
    # Linhas horizontais de referência
    lines_to_draw = [
        (14.47, "dm for obst6"),
        (11.72, "dm"),
        (1.9,   "colisão obst."),
        (4.55,  "colisão obst6")
    ]

    for (bag_file, robot_name) in bag_files:
        print(f"\nLendo {bag_file} (robô '{robot_name}') ...")
        start_time, end_time, dist_data = read_bag_and_calc_dist(bag_file)
        print(f"  - start_time = {start_time:.2f}, end_time = {end_time:.2f}")

        # 'dist_data' é { "obst1": [(t, dist), ...], "obst2": [...], ... }
        # Precisamos montar o plot para os 6 obstáculos
        # Eixo X: t - start_time
        # Eixo Y: dist
        # Se um obstáculo não tiver dados, não plota

        fig, ax = plt.subplots(figsize=(10,6))

        colors = ['blue','green','red','purple','brown','orange']
        # Se tiver + obst, pode acrescentar

        # Plotar cada obst
        i_color = 0
        plotted_any = False
        for obst_label in sorted(dist_data.keys()):
            if not dist_data[obst_label]:
                continue
            t_vals  = []
            d_vals  = []
            for (t_sim, dist) in dist_data[obst_label]:
                if t_sim >= start_time:
                    t_vals.append(t_sim - start_time)
                    d_vals.append(dist)
            if not t_vals:
                continue
            plotted_any = True
            color = colors[i_color % len(colors)]
            i_color += 1
            ax.plot(t_vals, d_vals,
                    label=obst_label,
                    color=color, linestyle='-')

        if not plotted_any:
            print("  [AVISO] Nenhuma distância < 41.18 encontrada após start_time.")
            plt.close(fig)
            continue

        # Linhas horizontais
        for (y_line, label_line) in lines_to_draw:
            ax.axhline(y=y_line, color='gray', linestyle='--', alpha=0.8)
            ax.text(0.5, y_line+0.2, label_line,
                    color='gray', ha='left', va='bottom')

        ax.set_xlabel("Tempo (s) desde start_time")
        ax.set_ylabel("Distância (m) até cada obstáculo (dist < 41.18)")
        ax.set_title(f"{robot_name} - Distância aos obstáculos vs Tempo")
        ax.legend()
        ax.grid(True)

        out_filename = f"dist_obst_{robot_name}.eps"
        plt.savefig(out_filename, format='eps')
        print(f"  -> Gráfico salvo como '{out_filename}'")
        plt.show()

    print("\nConcluído. Foram gerados gráficos para cada bag (quando havia dados < 41.18).")

# ============================================================================
if __name__ == "__main__":
    import matplotlib
    import matplotlib.pyplot as plt
    main()
