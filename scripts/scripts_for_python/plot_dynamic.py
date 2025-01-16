import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Nome do arquivo .bag a ser analisado
bag_file = "multi_tec1.bag"

# Tópico da força dinâmica
dynamic_force_topic = "/apfm/dynamic_force"

# Tópico do robô para descobrir o start_time
robot_topic = "/scenario/output_robot"

# Nome do arquivo de saída do gráfico
output_eps = "dynamic_force_normalized.eps"

def main():
    # 1) Ler a bag e coletar:
    #    - posição do robô p/ achar start_time
    #    - dados (t, fx, fy) da força dinâmica
    robot_positions = []
    dynamic_force_data = []

    current_sim_time = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(
            topics=[robot_topic, dynamic_force_topic, "/clock"]
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

            elif topic == dynamic_force_topic:
                if current_sim_time is None:
                    continue
                fx = msg.x
                fy = msg.y
                dynamic_force_data.append((current_sim_time, fx, fy))

    # Ordenar os dados pelo tempo
    robot_positions.sort(key=lambda x: x[0])
    dynamic_force_data.sort(key=lambda x: x[0])

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
    # raw_forces = (t_vals, mag_vals) (antes de normalizar)
    t_vals = []
    mag_vals = []
    for (t_sim, fx, fy) in dynamic_force_data:
        if t_sim >= start_time:
            # tempo relativo
            t_vals.append(t_sim - start_time)
            mag = np.hypot(fx, fy)
            mag_vals.append(mag)

    # ============================================================================
    # 4) Normalizar pela própria magnitude máxima
    # ============================================================================
    if mag_vals:
        max_mag = max(mag_vals)
        if max_mag > 1e-9:
            norm_vals = [m / max_mag for m in mag_vals]
        else:
            norm_vals = [0.0] * len(mag_vals)
    else:
        norm_vals = []

    # ============================================================================
    # 5) Plotar
    # ============================================================================
    if not t_vals:
        print("  [AVISO] Nenhum dado de força dinâmica após start_time.")
        return

    plt.figure(figsize=(10, 6))
    plt.plot(t_vals, norm_vals, label="dynamic", color="green", linestyle='-')

    plt.title(f"Força dinâmica normalizada (iniciando em t={start_time:.2f}s)")
    plt.xlabel("Tempo desde start_time (s)")
    plt.ylabel("Força normalizada")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(output_eps, format='eps')
    print(f"Gráfico salvo em '{output_eps}'.")
    plt.show()

    print("Concluído.")

if __name__ == "__main__":
    main()
