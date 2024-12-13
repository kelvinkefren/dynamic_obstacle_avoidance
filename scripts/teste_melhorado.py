import numpy as np
import matplotlib.pyplot as plt
import csv

def obstacle_angle(barco_pos, barco_vel, obst_pos):
    # Vetor posição relativo
    d = np.array(obst_pos) - np.array(barco_pos)
    
    # Normalizar direção da velocidade do barco
    barco_vel_unit = barco_vel / np.linalg.norm(barco_vel)
    
    # Ângulo entre o vetor relativo e a frente do barco
    dot_product = np.dot(d, barco_vel_unit)
    d_magnitude = np.linalg.norm(d)
    angle_rad = np.arccos(dot_product / d_magnitude)
    
    # Determinar o sentido do ângulo (horário/anti-horário)
    cross_z = d[0] * barco_vel_unit[1] - d[1] * barco_vel_unit[0]
    angle_deg = np.degrees(angle_rad)
    
    if cross_z < 0:
        angle_deg = 360 - angle_deg  # Ajuste para sentido horário
    
    return angle_deg

def cross_product_2d(v1, v2):
    # Calcula o produto vetorial 2D (apenas x e y)
    return v1[0] * v2[1] - v1[1] * v2[0]

def calculate_angle_between_vectors(v1, v2):
    # Calcula o ângulo entre dois vetores em 2D
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)
    if magnitude_v1 < 1e-8 or magnitude_v2 < 1e-8:
        return 0.0
    angle_rad = np.arccos(np.clip(dot_product / (magnitude_v1 * magnitude_v2), -1.0, 1.0))
    return np.degrees(angle_rad)

def check_collision_risk(barco_pos, barco_vel, obst_pos, obst_vel, dm):
    # Vetor relativo posição
    d = np.array(obst_pos) - np.array(barco_pos)
    
    # Velocidade relativa (barco em relação ao obstáculo)
    relative_vel = np.array(barco_vel) - np.array(obst_vel)
    
    if np.linalg.norm(relative_vel) < 1e-8:
        return False
    
    # Normalizar velocidade relativa
    rel_unit = relative_vel / np.linalg.norm(relative_vel)
    
    # Projeção do obstáculo na direção da velocidade relativa
    proj = np.dot(d, rel_unit)
    closest_point = np.array(barco_pos) + proj * rel_unit
    dist_to_line = np.linalg.norm((obst_pos - closest_point))
    
    if proj > 0 and dist_to_line <= dm:
        return True
    else:
        return False

def determine_avoidance(barco_pos, barco_vel, obst_pos, obst_vel,angle_between,angle_relative_to_line):
    # Parâmetros
    raio_barco = 2
    raio_obstaculo = 1
    safe_distance = 2
    dm = raio_barco + raio_obstaculo + safe_distance  # = 5
    CR = dm + 3*dm # = 20
    
    # Obter ângulo do obstáculo
    angle_to_obstacle = obstacle_angle(barco_pos, barco_vel, obst_pos)
    
    # Produto vetorial para determinar direita/esquerda
    cross_z = cross_product_2d(barco_vel, obst_vel)
    
    # Verificar risco de colisão
    risk_of_collision = check_collision_risk(barco_pos, barco_vel, obst_pos, obst_vel, dm)
    
    if risk_of_collision:
        # Classificar o tipo de encontro
        if (0 <= angle_to_obstacle <= 15) or (345 <= angle_to_obstacle <= 360):
            avoidance_type = "HeadsOn"
        elif 15 < angle_to_obstacle <= 112.5:
            avoidance_type = "Crossing A"
        elif 247.5 <= angle_to_obstacle < 345:
            avoidance_type = "Crossing B"
        elif 112.5 > angle_to_obstacle >= 247.5:
            avoidance_type = "Overtaking"
        else:
            avoidance_type = "No avoidance"
    else:
        avoidance_type = "No avoidance"

    # Correção para manobras erradas
    if avoidance_type in ["Crossing A", "Crossing B"]:
        if angle_relative_to_line > 30 and ((130 < angle_between < 180) or (0 < angle_between < 30)):
            avoidance_type = "No avoidance"
            print("Passou aqui")

    return avoidance_type, cross_z, dm, CR

def plot_positions_and_velocities_with_relative(barco_pos, barco_vel, obst_pos, obst_vel, dm, CR, title_suffix=""):
    """
    Plota as posições do barco, do obstáculo, as velocidades, vetores relativos,
    e as circunferências de segurança.
    """
    plt.figure(figsize=(6, 6))
    
    # Posição e vetor de velocidade do barco
    plt.quiver(barco_pos[0], barco_pos[1], barco_vel[0], barco_vel[1], 
               angles='xy', scale_units='xy', scale=1, color='blue', label='Barco')
    
    # Posição e vetor de velocidade do obstáculo
    plt.quiver(obst_pos[0], obst_pos[1], obst_vel[0], obst_vel[1], 
               angles='xy', scale_units='xy', scale=1, color='red', label='Obstáculo')
    
    # Destaque das posições
    plt.scatter(barco_pos[0], barco_pos[1], color='blue', s=50, label='Posição do Barco')
    plt.scatter(obst_pos[0], obst_pos[1], color='red', s=50, label='Posição do Obstáculo')
    
    # Linha entre o barco e o obstáculo
    plt.plot([barco_pos[0], obst_pos[0]], [barco_pos[1], obst_pos[1]], 'k--', label='Linha Relativa')
    
    # Velocidade relativa (barco em relação ao obstáculo)
    relative_velocity = np.array(barco_vel) - np.array(obst_vel)
    plt.quiver(barco_pos[0], barco_pos[1], relative_velocity[0], relative_velocity[1], 
               angles='xy', scale_units='xy', scale=1, color='green', label='Velocidade Relativa')
    
    # Ângulo entre o vetor relativo e a linha do barco ao obstáculo
    d = np.array(obst_pos) - np.array(barco_pos)
    angle_relative_to_line = calculate_angle_between_vectors(relative_velocity, d)
    
    # Ajustar o sentido com base nas condições
    cross_z = cross_product_2d(barco_vel, obst_vel)
    dot_product = np.dot(barco_vel, obst_vel)
    angle_between = calculate_angle_between_vectors(barco_vel, obst_vel)
    sentido = "horário" if cross_z > 0 else "anti-horário"
    
    # Exibir resultados no gráfico
    plt.text(-10, -8, f"Produto Vetorial: {cross_z:.2f}", color='orange')
    plt.text(-10, -9, f"Produto Escalar: {dot_product:.2f}", color='green')
    plt.text(-10, -10, f"Ângulo: {angle_between:.2f}°", color='purple')
    plt.text(-10, -11, f"Sentido: {sentido}", color='blue')
    plt.text(-10, -12, f"Ângulo Relativo: {angle_relative_to_line:.2f}°", color='brown')
    
    # Plotar circunferência de raio dm ao redor do obstáculo
    circle_obst = plt.Circle((obst_pos[0], obst_pos[1]), dm, color='r', fill=False, linestyle='--', label='Área de dm')
    plt.gca().add_patch(circle_obst)
    
    # Plotar circunferência de raio CR ao redor do barco
    circle_barco = plt.Circle((barco_pos[0], barco_pos[1]), CR, color='b', fill=False, linestyle='--', label='Área de CR')
    plt.gca().add_patch(circle_barco)
    
    # Configuração do gráfico
    plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
    plt.grid(color='gray', linestyle='--', linewidth=0.5)
    plt.legend()
    plt.title(f"Simulação: {title_suffix}")
    plt.xlabel("Posição X")
    plt.ylabel("Posição Y")
    plt.axis('equal')  # Manter a escala igual para X e Y
    plt.show()

# Parâmetros fixos do barco
barco_pos = [0, 0]
barco_vel = [1.0, 0]

# Diferentes posições do obstáculo
obst_positions = [
    [15, -4],
    [8, 2],
    [14, -5],
    [4, -6.5],
    [14, 5],
    [4, 6.5]
]

# Conjunto de velocidades do obstáculo: 2 velocidades e suas opostas
obst_vel_candidates = [
    [0.6, 0.7],
    [2.0, -1.0],
    [0.1, 0.8],
    [0.8,0.1]
]

# Armazenar resultados
results = []

# Executar simulações
sim_count = 1
for pos in obst_positions:
    for vel in obst_vel_candidates:
        # Velocidade original
        
        angle_between = calculate_angle_between_vectors(barco_vel, vel)
        relative_velocity = np.array(barco_vel) - np.array(vel)
        angle_relative_to_line = calculate_angle_between_vectors(relative_velocity, np.array(pos) - np.array(barco_pos))
        avoidance_type, cross_z, dm, CR = determine_avoidance(barco_pos, barco_vel, pos, vel,angle_between,angle_relative_to_line)
        dot_product = np.dot(barco_vel, vel)
        sentido = "horário" if cross_z > 0 else "anti-horário"

        results.append([pos, vel, cross_z, dot_product, angle_between, sentido, avoidance_type, angle_relative_to_line])
        # plot_positions_and_velocities_with_relative(barco_pos, barco_vel, pos, vel, dm, CR,
        #                                             title_suffix=f"Pos={pos}, Vel={vel}, Tipo={avoidance_type}")
        sim_count += 1
        
        # Velocidade oposta
        vel_oposta = [-vel[0], -vel[1]]
        angle_between = calculate_angle_between_vectors(barco_vel, vel_oposta)
        relative_velocity = np.array(barco_vel) - np.array(vel_oposta)
        angle_relative_to_line = calculate_angle_between_vectors(relative_velocity, np.array(pos) - np.array(barco_pos))
        avoidance_type, cross_z, dm, CR = determine_avoidance(barco_pos, barco_vel, pos, vel_oposta,angle_between,angle_relative_to_line)
        dot_product = np.dot(barco_vel, vel_oposta)
        sentido = "horário" if cross_z > 0 else "anti-horário"

        results.append([pos, vel_oposta, cross_z, dot_product, angle_between, sentido, avoidance_type, angle_relative_to_line])
        # plot_positions_and_velocities_with_relative(barco_pos, barco_vel, pos, vel_oposta, dm, CR,
        #                                             title_suffix=f"Pos={pos}, Vel={vel_oposta}, Tipo={avoidance_type}")
        sim_count += 1

# Salvar resultados em CSV
with open('simulation_results.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(["Posicao", "Velocidade", "Cross_Z", "Dot_Product", "Angle_Between", "Sentido", "Avoidance_Type", "Angle_Relative_To_Line"])
    csvwriter.writerows(results)
