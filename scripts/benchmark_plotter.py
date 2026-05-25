#!/usr/bin/env python3
"""
benchmark_plotter.py
Plota as trajetórias extraídas no arquivo _trajectory.csv por cima 
da arquitetura (pontos chaves) para inclusão no artigo do autor.

Uso: python3 benchmark_plotter.py meu_bag_trajectory.csv
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import yaml

def parse_waypoints(yaml_path="params/waypoints.yaml"):
    try:
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            params = data['/**']['ros__parameters']
            waypoints = {}
            for key, val in params.items():
                if isinstance(val, list) and len(val) >= 2:
                    waypoints[key] = (val[0], val[1])
            return waypoints
    except Exception as e:
        print(f"Aviso: Não foi possível carregar waypoints.yaml ({e})")
        return {}


def plot_trajectory(csv_file):
    if not os.path.exists(csv_file):
        print(f"Erro: Arquivo CSV {csv_file} não encontrado.")
        return

    # Lendo odometria acadêmica limpa do bag via pandas
    df = pd.read_csv(csv_file)
    
    # Criar uma Figura estilizada para Paper Acadêmico IEEE / Springer
    # Formato 1 coluna: largura ~3.5 polegadas
    fig, ax = plt.subplots(figsize=(6, 5))
    
    # 1. Tracar Linha Continua Verde da Trajetoria (AMCL)
    ax.plot(df['x'], df['y'], color='#2ca02c', linewidth=1.8, label="Robot Trajectory (AMCL)")
    
    # 2. Adicionar ponto central de início e fim da gravação (em azul e vermelho)
    ax.scatter(df['x'].iloc[0], df['y'].iloc[0], color='blue', s=80, marker='o', label="Start", zorder=5)
    ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='red', s=80, marker='X', label="End", zorder=5)

    # 3. Desenhar os "Prédios" / Waypoints teóricos por baixo como referência
    waypoints = parse_waypoints()
    if waypoints:
        wp_x = [v[0] for v in waypoints.values()]
        wp_y = [v[1] for v in waypoints.values()]
        ax.scatter(wp_x, wp_y, color='black', s=20, marker='s', alpha=0.5, label="Map Waypoints")
        
        # Etiquetar nome curto dos the points if wanted
        for name, (x, y) in waypoints.items():
            label_name = name.replace('_point', '').replace('_station', '').capitalize()
            ax.annotate(label_name, (x, y), xytext=(5, 5), textcoords='offset points', 
                        fontsize=8, alpha=0.7)

    # 4. Formatações exigidas em gráficos de mestrado/tcc/artigo
    ax.set_title("Autonomous Delivery Trajectory Evaluation", pad=15, fontsize=12, fontweight='bold')
    ax.set_xlabel("Map X Coordinate (m)", fontsize=10)
    ax.set_ylabel("Map Y Coordinate (m)", fontsize=10)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc="lower right", fontsize=8)
    
    ax.set_aspect('equal', adjustable='datalim')
    plt.tight_layout()

    # Define nome de saida e salva vetorizado e rasterizado
    output_pdf = csv_file.replace('.csv', '.pdf')
    output_png = csv_file.replace('.csv', '.png')
    
    plt.savefig(output_pdf, bbox_inches='tight', dpi=300)
    plt.savefig(output_png, bbox_inches='tight', dpi=300)
    
    print(f"Gráfico Final Gerado: {output_pdf} e {output_png}")
    plt.close()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Uso: python3 benchmark_plotter.py <arquivo_trajectory.csv>")
        print("Ou: python3 benchmark_plotter.py --all  (Lê todos CSV da pasta atual)")
        sys.exit(1)
        
    arg = sys.argv[1]
    import glob
    if arg == '--all':
        csvs = glob.glob('*_trajectory.csv')
        print(f"Gerando gráficos para {len(csvs)} baterias.")
        for c in csvs:
            plot_trajectory(c)
    else:
        plot_trajectory(arg)
