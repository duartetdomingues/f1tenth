import pandas as pd
import matplotlib.pyplot as plt
import argparse
import numpy as np

def plot_trajectory_with_yaw(csv_file):
    # Ler o arquivo CSV usando pandas
    data = pd.read_csv(csv_file)

    # Verificar se as colunas esperadas existem
    if 'x' not in data.columns or 'y' not in data.columns or 'yaw' not in data.columns:
        print("Erro: O arquivo CSV não contém as colunas esperadas (x, y, yaw).")
        return

    # Plotar a trajetória
    plt.figure(figsize=(10, 8))
    plt.plot(data['x'], data['y'], label="Trajetória", color='b', linewidth=2)

    # Plotar setas representando o yaw
    for i in range(0, len(data), int(len(data) / 10)):  # Apenas plotando uma seta a cada 10 pontos
        x = data['x'][i]
        y = data['y'][i]
        yaw = data['yaw'][i]
        
        # Cálculo do vetor direcional baseado no yaw
        dx = 0.2 * np.cos(yaw)  # Deslocamento no eixo x (ajustável)
        dy = 0.2 * np.sin(yaw)  # Deslocamento no eixo y (ajustável)
        
        # Desenhando a seta para representar a direção
        plt.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='r', ec='r')

    # Adicionar rótulos e título
    plt.title(f"Trajetória com Yaw", fontsize=16)
    plt.xlabel('x', fontsize=12)
    plt.ylabel('y', fontsize=12)
    
    # Adicionar grade
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')  # Para garantir que o gráfico seja proporcional

    # Adicionar legenda
    plt.legend()

    # Exibir o gráfico
    plt.show()

def main():
    # Configuração para receber o argumento do caminho do arquivo CSV
    parser = argparse.ArgumentParser(description='Plotar trajetória com yaw a partir de um arquivo CSV.')
    parser.add_argument('csv_file', type=str, help='Caminho do arquivo CSV contendo os dados da trajetória')

    # Obter o caminho do arquivo a partir dos argumentos
    args = parser.parse_args()

    # Plotar a trajetória com yaw usando o arquivo CSV fornecido
    plot_trajectory_with_yaw(args.csv_file)

if __name__ == "__main__":
    main()
