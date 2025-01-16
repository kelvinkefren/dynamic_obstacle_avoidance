import rosbag
import os
import sys

# Lista de arquivos .bag para análise
bag_files = [
    "bag/dissertacao.bag",           # 1
    "bag/dissertacao_2.bag",         # 2
    "bag/dissertacao_3.bag",         # 3
    "bag/dissertacao_teste.bag",     # 4
    "bag/dissertacao_teste_2.bag",   # 5
    "bag/dissertacao_teste_3.bag"    # 6
]

# Tópicos de interesse
collision_topic = "/obstacle_avoidance/collision"
scenario_topic = "/current_scenario"

# Função para analisar um único arquivo .bag
def analyze_bag(file_path, collision_topic, scenario_topic):
    true_count = 0
    unique_scenarios_on_true = set()
    current_scenario = None
    
    # Abrir o arquivo .bag
    with rosbag.Bag(file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[collision_topic, scenario_topic]):
            if topic == scenario_topic:  # Atualiza o cenário atual
                current_scenario = msg.data
            elif topic == collision_topic and msg.data:  # Se a mensagem for True
                true_count += 1
                if current_scenario:  # Salva o cenário correspondente
                    unique_scenarios_on_true.add(current_scenario)
    
    return true_count, unique_scenarios_on_true

def main():
    # Verifica se um argumento foi passado
    if len(sys.argv) != 2:
        print("Uso: python3 code.py <número_do_arquivo>")
        print("Exemplo: python3 code.py 4 para processar 'bag/dissertacao_teste.bag'")
        sys.exit(1)
    
    try:
        # Tenta converter o argumento para inteiro
        file_number = int(sys.argv[1])
    except ValueError:
        print("Erro: O argumento deve ser um número inteiro.")
        sys.exit(1)
    
    # Verifica se o número está dentro do intervalo válido
    if not 1 <= file_number <= len(bag_files):
        print(f"Erro: O número deve estar entre 1 e {len(bag_files)}.")
        sys.exit(1)
    
    # Obtém o arquivo correspondente (1-based index)
    selected_file = bag_files[file_number - 1]
    
    if not os.path.exists(selected_file):
        print(f"Arquivo {selected_file} não encontrado.")
        sys.exit(1)
    
    print(f"Analisando o arquivo: {selected_file}")
    true_count, unique_scenarios = analyze_bag(selected_file, collision_topic, scenario_topic)
    
    # Exibir resultados do arquivo selecionado
    print(f"Número de True no tópico '{collision_topic}' no arquivo {selected_file}: {true_count}")
    print("Cenários únicos quando houve True neste arquivo:")
    for scenario in sorted(unique_scenarios):  # Ordena os cenários para exibição
        print(f"  - {scenario}")
    print("-" * 50)
    
    # Exibir resumo final consolidado
    print("\nResumo final consolidado:")
    print("Número de True por arquivo:")
    collision_results = {}
    all_unique_scenarios_on_true = set()
    
    # Apenas o arquivo selecionado está sendo processado, então adicionamos seus resultados
    collision_results[selected_file] = true_count
    all_unique_scenarios_on_true.update(unique_scenarios)
    
    for bag_file, true_count in collision_results.items():
        print(f"  {bag_file}: {true_count} vezes")
    
    print("\nTodos os cenários únicos quando houve True (consolidado):")
    for scenario in sorted(all_unique_scenarios_on_true):  # Ordena os cenários para exibição
        print(f"  - {scenario}")

if __name__ == "__main__":
    main()
