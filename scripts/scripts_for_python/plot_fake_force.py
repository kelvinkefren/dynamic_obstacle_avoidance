#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Configurações
bag1 = "multi_tec1.bag"  # Robô 1 (migbot_kel)
bag2 = "multi_tec2.bag"  # Robô 2 (migbot_lyu)

robot_topic = "/scenario/output_robot"

force_topics = [
    ("/apfm/attractive_force", "attractive"),
    ("/apfm/dynamic_force",    "dynamic"),
    ("/apfm/emergency_force",  "emergency"),
    ("/apfm/static_force",     "static"),
]

SUBSTITUTION_TIME = 5.0  # 5 segundos (onde faremos a substituição do dynamic)

def read_bag_and_normalize(bag_file):
    """
    Lê bag_file, descobre start_time (robô mais perto de (0,0)),
    e normaliza as quatro forças (attractive, dynamic, emergency, static).
    Retorna: (start_time, normalized_forces),
        onde normalized_forces[label] = (t_vals, norm_vals)
        com t_vals >= start_time (convertidos para t - start_time).
    """
    # Ler dados
    robot_positions = []
    force_data = { label: [] for (_, label) in force_topics }

    current_sim_time = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(
            topics=[robot_topic] + [f[0] for f in force_topics] + ["/clock"]
        ):
            if topic == "/clock":
                current_sim_time = msg.clock.to_sec()
            elif topic == robot_topic:
                if current_sim_time is not None:
                    rx = msg.position.x
                    ry = msg.position.y
                    robot_positions.append((current_sim_time, rx, ry))
            else:
                for (topic_name, label) in force_topics:
                    if topic == topic_name and current_sim_time is not None:
                        fx = msg.x
                        fy = msg.y
                        force_data[label].append((current_sim_time, fx, fy))
                        break

    # Ordenar
    robot_positions.sort(key=lambda x: x[0])
    for lbl in force_data:
        force_data[lbl].sort(key=lambda x: x[0])

    # Determinar start_time
    if not robot_positions:
        raise RuntimeError(f"Nenhuma posição do robô em {bag_file}.")
    min_dist = float('inf')
    start_time = None
    for (t_sim, rx, ry) in robot_positions:
        d0 = np.hypot(rx, ry)
        if d0 < min_dist:
            min_dist = d0
            start_time = t_sim

    # Calcular magnitudes e normalizar cada força
    normalized_forces = {}
    for lbl in force_data:
        t_vals = []
        mag_vals = []
        for (t_sim, fx, fy) in force_data[lbl]:
            if t_sim >= start_time:
                t_rel = t_sim - start_time
                mag = np.hypot(fx, fy)
                t_vals.append(t_rel)
                mag_vals.append(mag)
        if not mag_vals:
            normalized_forces[lbl] = ([], [])
            continue
        max_mag = max(mag_vals)
        if max_mag > 1e-9:
            norm_vals = [m / max_mag for m in mag_vals]
        else:
            norm_vals = [0.0]*len(mag_vals)
        normalized_forces[lbl] = (t_vals, norm_vals)

    return start_time, normalized_forces

def plot_forces(robot_name, normalized_forces, out_file):
    """
    Plota as quatro forças normalizadas e salva em out_file (EPS).
    """
    plt.figure(figsize=(10,6))
    color_map = {
        "attractive": "blue",
        "dynamic":    "green",
        "emergency":  "red",
        "static":     "orange",
    }

    plotted_any = False
    for lbl in ["attractive","dynamic","emergency","static"]:
        if lbl not in normalized_forces:
            continue
        t_vals, v_vals = normalized_forces[lbl]
        if not t_vals:
            continue
        plotted_any = True
        plt.plot(t_vals, v_vals, label=lbl, color=color_map.get(lbl,"gray"))

    if not plotted_any:
        print(f"  [AVISO] Não há dados de força para plotar em {robot_name}.")
        return

    plt.title(f"Forças Normalizadas - {robot_name}")
    plt.xlabel("Tempo (s) desde start_time")
    plt.ylabel("Força normalizada (0..1)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(out_file, format='eps')
    print(f"  -> Gráfico salvo como '{out_file}'")
    plt.show()

def substitute_dynamic_force(
    tec1_forces,  # normalized_forces de tec1
    tec2_forces,  # normalized_forces de tec2
    substitution_time
):
    """
    Substitui a força 'dynamic' de tec1_forces nos primeiros 'substitution_time' segundos
    pelos valores de tec2_forces.
    """
    # Copiar dicionário
    updated_forces = {}
    for lbl in tec1_forces:
        # simplesmente copia
        updated_forces[lbl] = (
            list(tec1_forces[lbl][0]),
            list(tec1_forces[lbl][1])
        )

    # Focar apenas na 'dynamic'
    # Precisamos do dynamic em tec1 e tec2
    if 'dynamic' not in tec1_forces or 'dynamic' not in tec2_forces:
        print("  [ERRO] 'dynamic' não está em tec1 ou tec2.")
        return updated_forces

    t1, v1 = updated_forces['dynamic']  # list
    t2, v2 = tec2_forces['dynamic']     # original data
    # Converter para np.array para facilitar
    t1_arr = np.array(t1)
    v1_arr = np.array(v1)
    t2_arr = np.array(t2)
    v2_arr = np.array(v2)

    # Filtrar indices no tec1 (onde t1 <= substitution_time)
    idx_t1 = np.where(t1_arr <= substitution_time)[0]
    # Filtrar indices no tec2 (onde t2 <= substitution_time)
    idx_t2 = np.where(t2_arr <= substitution_time)[0]

    # Substituir até min(len(idx_t1), len(idx_t2)) amostras
    count_sub = min(len(idx_t1), len(idx_t2))
    for i in range(count_sub):
        # Posição i do array de indices
        i1 = idx_t1[i]
        i2 = idx_t2[i]
        v1_arr[i1] = v2_arr[i2]

    updated_forces['dynamic'] = (t1_arr.tolist(), v1_arr.tolist())
    return updated_forces

def main():
    # 1) Ler e normalizar tec1
    start1, norm1 = read_bag_and_normalize(bag1)
    # 2) Ler e normalizar tec2
    start2, norm2 = read_bag_and_normalize(bag2)

    # 3) Substituir dynamic do tec1 pelos primeiros 5s do tec2
    #    antes de plottar tec1
    updated_tec1 = substitute_dynamic_force(norm1, norm2, SUBSTITUTION_TIME)

    # 4) Plotar tec1 (já com dynamic_tec2 nos primeiros 5s)
    out1 = f"forces_sub_dynamic_migbot_kel.eps"
    plot_forces("migbot_kel ", updated_tec1, out1)

    # 5) Plotar tec2 normal
    out2 = f"forces_normalized_migbot_lyu.eps"
    plot_forces("migbot_lyu", norm2, out2)

    print("Concluído.")

if __name__ == "__main__":
    main()
