#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

# Nome do arquivo .bag a ser analisado
bag_file = "multi_tec1.bag"

# Tópicos de força que vamos analisar (ignorando total_force)
force_topics = [
    ("/apfm/attractive_force", "attractive"),
    ("/apfm/dynamic_force",    "dynamic"),
    ("/apfm/emergency_force",  "emergency"),
    ("/apfm/static_force",     "static"),
]

# Tópico do robô para descobrir o start_time
robot_topic = "/scenario/output_robot"

# Nome do arquivo de saída do gráfico
output_eps = "forces_normalized.eps"


def main():
    # 1) Ler a bag e coletar:
    #    - posição do robô p/ achar start_time
    #    - dados (t, fx, fy) de cada força
    robot_positions = []
    force_data = {}  # { label: [(t, fx, fy), ...] }
    for (_, label) in force_topics:
        force_data[label] = []

    current_sim_time = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(
            topics=[robot_topic] + [ft[0] for ft in force_topics] + ["/clock"]
        ):
            if topic == "/clock":
                # clock -> msg.clock.to_sec()
                current_sim_time = msg.clock.to_sec()

            elif topic == robot_topic:
                if current_sim_time is None:
                    continue
                # posição do robô
                rx = msg.position.x
                ry = msg.position.y
                robot_positions.append((current_sim_time, rx, ry))

            else:
                # Verificar se é um dos tópicos de força
                for (force_topic_name, force_label) in force_topics:
                    if topic == force_topic_name:
                        if current_sim_time is None:
                            continue
                        fx = msg.x
                        fy = msg.y
                        force_data[force_label].append((current_sim_time, fx, fy))
                        break

    # Ordenar os dados pelo tempo
    robot_positions.sort(key=lambda x: x[0])
    for label in force_data:
        force_data[label].sort(key=lambda x: x[0])

    # Se não houver dados do robô, erro
    if not robot_positions:
        raise RuntimeError(f"Nenhuma posição do robô encontrada em {bag_file}.")

    # ============================================================================
    # 2) Determinar start_time (quando o robô está mais perto de (0,0))
    # ============================================================================
    min_dist = float('inf')
    start_time = None
    for (t_sim, rx, ry) in robot_positions:
        d0 = np.hypot(rx, ry)
        if d0 < min_dist:
            min_dist = d0
            start_time = t_sim

    print(f"Arquivo: {bag_file}")
    print(f"  start_time = {start_time:.2f} (dist mínima = {min_dist:.2f})")

    
    # ============================================================================
    # 3) Calcular a magnitude e filtrar tempos >= start_time
    # ============================================================================
    # raw_forces[label] = (t_vals, mag_vals)  (antes de normalizar)
    raw_forces = {}

    for label in force_data:
        t_vals = []
        mag_vals = []
        for (t_sim, fx, fy) in force_data[label]:
            if t_sim >= start_time:
                # tempo relativo
                t_vals.append(t_sim - start_time)
                mag = np.hypot(fx, fy)
                mag_vals.append(mag)
        raw_forces[label] = (t_vals, mag_vals)

    # ============================================================================
    # 4) Normalizar cada força pelo seu próprio máximo
    # ============================================================================
    normalized_forces = {}
    for label in raw_forces:
        (t_vals, mag_vals) = raw_forces[label]
        if not mag_vals:
            # se não houver dados após start_time
            normalized_forces[label] = ([], [])
            continue

        max_mag = max(mag_vals)
        if max_mag > 1e-9:
            norm_vals = [m / max_mag for m in mag_vals]
        else:
            norm_vals = [0.0]*len(mag_vals)

        normalized_forces[label] = (t_vals, norm_vals)

    # ============================================================================
    # 5) Plotar
    # ============================================================================
    plt.figure(figsize=(10,6))

    color_map = {
        "attractive": "blue",
        "dynamic":    "green",
        "emergency":  "red",
        "static":     "orange",
    }

    plotted_any = False
    for label in ["attractive", "dynamic", "emergency", "static"]:
        (t_vals, norm_vals) = normalized_forces[label]
        if not t_vals:
            print(f"  [AVISO] Força '{label}' não teve dados após start_time.")
            continue
        plotted_any = True
        plt.plot(t_vals, norm_vals,
                 label=label,
                 color=color_map.get(label, "gray"),
                 linestyle='-')

    if not plotted_any:
        print("Não houve dados de força para plotar.")
        return

    plt.title(f"Forças normalizadas (iniciando em t={start_time:.2f}s)")
    plt.xlabel("Tempo desde start_time (s)")
    plt.ylabel("Força normalizada (cada por seu máximo)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(output_eps, format='eps')
    print(f"Gráfico salvo em '{output_eps}'.")
    plt.show()

    print("Concluído.")


if __name__ == "__main__":
    main()
