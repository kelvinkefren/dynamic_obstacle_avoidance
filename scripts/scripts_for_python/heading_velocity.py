#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import euler_from_quaternion  # Certifique-se de ter o pacote `tf` instalado
import os

# ============================================================================
# CONFIGURAÇÕES INICIAIS
# ============================================================================

# Lista de arquivos .bag e seus respectivos nomes de robô
bag_files = [
    ("multi_tec1.bag", "migbot_kel"),
    ("multi_tec2.bag", "migbot_lyu"),
    ("multi_tec3.bag", "migbot_apf")
]

# Tópico a ser analisado
robot_topic = "/scenario/output_robot"

# Nome do arquivo de saída do gráfico
output_eps_template = "heading_velocity_normalized_{}.eps"  # {} será substituído pelo nome do robô

# Tempo de substituição em segundos
SUBSTITUTION_TIME = 5.0  # 5 segundos

# ============================================================================
# FUNÇÕES AUXILIARES
# ============================================================================

def read_bag_and_extract_data(bag_file, robot_name):
    """
    Lê um arquivo .bag, encontra start_time (robô mais próximo de (0,0)),
    e coleta os dados de heading e velocidade.

    Retorna:
        - start_time (float): instante de início
        - heading_data (list of tuples): [(t_rel, yaw_deg), ...]
        - velocity_data (list of tuples): [(t_rel, norm_speed), ...]
        - max_speed (float): velocidade máxima observada
    """
    robot_positions = []
    heading_data = []
    velocity_data = []

    current_sim_time = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[robot_topic, "/clock"]):
            if topic == "/clock":
                current_sim_time = msg.clock.to_sec()
            elif topic == robot_topic:
                if current_sim_time is None:
                    continue
                # Posição do robô
                rx = msg.position.x
                ry = msg.position.y
                # Velocidade do robô
                vx = msg.velocity.x
                vy = msg.velocity.y
                # Orientação do robô (quaternion)
                qx = msg.orientation.x
                qy = msg.orientation.y
                qz = msg.orientation.z
                qw = msg.orientation.w

                robot_positions.append((current_sim_time, rx, ry))

                # Calcular heading (yaw) em graus
                quaternion = [qx, qy, qz, qw]
                euler = euler_from_quaternion(quaternion)
                yaw = euler[2]  # Yaw é o terceiro elemento
                yaw_degrees = np.degrees(yaw)

                # Calcular velocidade (magnitude)
                speed = np.hypot(vx, vy)

                # Armazenar os dados
                heading_data.append((current_sim_time, yaw_degrees))
                velocity_data.append((current_sim_time, speed))

    # Ordenar os dados pelo tempo
    robot_positions.sort(key=lambda x: x[0])
    heading_data.sort(key=lambda x: x[0])
    velocity_data.sort(key=lambda x: x[0])

    if not robot_positions:
        raise RuntimeError(f"Nenhuma posição do robô encontrada em {bag_file}.")

    # Determinar start_time (quando o robô está mais próximo de (0,0))
    min_dist = float('inf')
    start_time = None
    for (t_sim, rx, ry) in robot_positions:
        dist = np.hypot(rx, ry)
        if dist < min_dist:
            min_dist = dist
            start_time = t_sim

    print(f"Arquivo: {bag_file}")
    print(f"  start_time = {start_time:.2f} s (distância mínima = {min_dist:.2f} m)")

    # Filtrar dados a partir do start_time e calcular tempo relativo
    heading_rel = []
    speed_rel = []

    # Extração de heading
    for (t_sim, yaw_deg) in heading_data:
        if t_sim >= start_time:
            t_rel = t_sim - start_time
            heading_rel.append((t_rel, yaw_deg))

    # Extração de velocidade
    for (t_sim, speed) in velocity_data:
        if t_sim >= start_time:
            t_rel = t_sim - start_time
            speed_rel.append((t_rel, speed))

    # Encontrar velocidade máxima
    speed_vals = [speed for (_, speed) in speed_rel]
    if speed_vals:
        max_speed = max(speed_vals)
    else:
        max_speed = 0.0  # Evitar divisão por zero

    # Normalizar velocidade
    velocity_normalized = []
    for (t_rel, speed) in speed_rel:
        norm_speed = speed / max_speed if max_speed > 1e-9 else 0.0
        velocity_normalized.append((t_rel, norm_speed))

    # Heading não é normalizado, pois é um ângulo
    heading_normalized = heading_rel.copy()

    return start_time, heading_normalized, velocity_normalized, max_speed

def substitute_first_seconds(target_data, source_data, substitution_time=5.0):
    """
    Substitui os primeiros 'substitution_time' segundos dos dados alvo com os dados de origem.

    Args:
        - target_data (list of tuples): [(t_rel, value), ...]
        - source_data (list of tuples): [(t_rel, value), ...]
        - substitution_time (float): tempo em segundos para substituição

    Retorna:
        - updated_target_data (list of tuples): dados com substituição realizada
    """
    # Converter listas para arrays para facilitar a manipulação
    target_times = np.array([t for (t, _) in target_data])
    target_vals = np.array([v for (_, v) in target_data])

    source_times = np.array([t for (t, _) in source_data])
    source_vals = np.array([v for (_, v) in source_data])

    # Identificar índices onde t <= substitution_time
    target_inds = np.where(target_times <= substitution_time)[0]
    source_inds = np.where(source_times <= substitution_time)[0]

    # Determinar quantos pontos podem ser substituídos
    count_sub = min(len(target_inds), len(source_inds))
    if count_sub == 0:
        print("  [AVISO] Não há dados suficientes para substituição.")
        return target_data.copy()

    # Substituir os valores
    target_vals[target_inds[:count_sub]] = source_vals[source_inds[:count_sub]]

    # Reconstruir a lista de tuples
    updated_target_data = list(zip(target_times, target_vals))

    return updated_target_data

def plot_heading_velocity(robot_name, heading_data, velocity_data, max_speed, output_eps):
    """
    Plota heading e velocidade em subplots separados.

    Args:
        - robot_name (str): nome do robô para o título do gráfico
        - heading_data (list of tuples): [(t_rel, yaw_deg), ...]
        - velocity_data (list of tuples): [(t_rel, norm_speed), ...]
        - max_speed (float): velocidade máxima observada
        - output_eps (str): nome do arquivo de saída (EPS)
    """
    plt.figure(figsize=(12, 8))

    # Subplot para Heading
    plt.subplot(2, 1, 1)
    if heading_data:
        t_heading = [t for (t, _) in heading_data]
        yaw_deg = [v for (_, v) in heading_data]
        plt.plot(t_heading, yaw_deg, label="Heading", color="blue")
        plt.title(f"{robot_name} - Heading\nVelocidade Máxima: {max_speed:.2f} m/s")
        plt.xlabel("Tempo desde start_time (s)")
        plt.ylabel("Heading (°)")
        plt.grid(True)
        plt.legend()
    else:
        plt.text(0.5, 0.5, "Nenhum dado de Heading disponível.", 
                 horizontalalignment='center', verticalalignment='center')
        plt.axis('off')

    # Subplot para Velocidade
    plt.subplot(2, 1, 2)
    if velocity_data:
        t_velocity = [t for (t, _) in velocity_data]
        norm_speed = [v for (_, v) in velocity_data]
        plt.plot(t_velocity, norm_speed, label="Velocidade Normalizada", color="green")
        plt.title(f"{robot_name} - Velocidade Normalizada")
        plt.xlabel("Tempo desde start_time (s)")
        plt.ylabel("Velocidade Normalizada (0..1)")
        plt.grid(True)
        plt.legend()
        # Adicionar anotação da velocidade máxima no gráfico
        plt.annotate(f"Velocidade Máxima: {max_speed:.2f} m/s",
                     xy=(0.05, 0.95), xycoords='axes fraction',
                     fontsize=10, backgroundcolor='white',
                     verticalalignment='top')
    else:
        plt.text(0.5, 0.5, "Nenhum dado de Velocidade disponível.", 
                 horizontalalignment='center', verticalalignment='center')
        plt.axis('off')

    plt.tight_layout()
    plt.savefig(output_eps, format='eps')
    print(f"  -> Gráfico salvo como '{output_eps}'.")
    plt.show()

def main():
    # Armazenar os primeiros 5 segundos de cada bag para substituição
    # Neste caso, vamos substituir:
    # - Os primeiros 5s de migbot_kel com migbot_lyu
    # - Os primeiros 5s de migbot_lyu com migbot_apf
    # - migbot_apf permanece inalterado

    # Dicionários para armazenar os dados
    bags_data = {}
    substitutions = {}  # key: target_robot, value: source_robot

    # Definir a cadeia de substituição
    substitutions = {
        "migbot_kel": "migbot_lyu",
        "migbot_lyu": "migbot_apf"
        # "migbot_apf": None  # Não há substituição para migbot_apf
    }

    # Primeiro, ler e armazenar todos os dados
    for bag_file, robot_name in bag_files:
        print(f"\nProcessando '{bag_file}' para o robô '{robot_name}'...")
        try:
            start_time, heading_normalized, velocity_normalized, max_speed = read_bag_and_extract_data(bag_file, robot_name)
            bags_data[robot_name] = {
                "start_time": start_time,
                "heading": heading_normalized,
                "velocity": velocity_normalized,
                "max_speed": max_speed
            }
        except RuntimeError as e:
            print(f"  [ERRO] {e}")
            continue

    # Realizar as substituições conforme a cadeia definida
    for target_robot, source_robot in substitutions.items():
        if target_robot not in bags_data:
            print(f"  [AVISO] Dados do robô alvo '{target_robot}' não disponíveis.")
            continue
        if source_robot not in bags_data:
            print(f"  [AVISO] Dados do robô fonte '{source_robot}' não disponíveis.")
            continue

        # Substituir Heading
        updated_heading = substitute_first_seconds(
            bags_data[target_robot]["heading"],
            bags_data[source_robot]["heading"],
            substitution_time=SUBSTITUTION_TIME
        )

        # Substituir Velocidade
        updated_velocity = substitute_first_seconds(
            bags_data[target_robot]["velocity"],
            bags_data[source_robot]["velocity"],
            substitution_time=SUBSTITUTION_TIME
        )

        # Atualizar os dados do target_robot
        bags_data[target_robot]["heading"] = updated_heading
        bags_data[target_robot]["velocity"] = updated_velocity

        print(f"  -> Substituição dos primeiros {SUBSTITUTION_TIME} segundos de '{target_robot}' com dados de '{source_robot}' realizada.")

    # Agora, plotar os gráficos em ordem: migbot_kel, migbot_lyu, migbot_apf
    for bag_file, robot_name in bag_files:
        if robot_name not in bags_data:
            print(f"\n[AVISO] Não há dados para o robô '{robot_name}'.")
            continue
        output_eps = output_eps_template.format(robot_name)
        print(f"\nPlotando dados para o robô '{robot_name}'...")
        plot_heading_velocity(
            robot_name, 
            bags_data[robot_name]["heading"], 
            bags_data[robot_name]["velocity"], 
            bags_data[robot_name]["max_speed"], 
            output_eps
        )

    print("\nConcluído. Todos os gráficos foram gerados e salvos.")

# ============================================================================
# EXECUTAR MAIN
# ============================================================================

if __name__ == "__main__":
    main()
