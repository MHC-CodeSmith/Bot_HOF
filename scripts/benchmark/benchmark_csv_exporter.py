#!/usr/bin/env python3
"""
benchmark_csv_exporter.py
Extrai dados de navegação de um rosbag2 (SQLite3 db3) para um arquivo CSV.
Ideal para análise offline no Excel, R ou Pandas para artigos acadêmicos.

Dependências não-ROS:
  pip install rosbags pandas
"""

import sys
import os
import csv
import argparse
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def export_bag_to_csv(bag_path):
    bag_name = os.path.basename(os.path.normpath(bag_path))
    csv_file = f"{bag_name}_trajectory.csv"
    
    print(f"Lendo Bag: {bag_path}")
    
    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Cabeçalho do CSV
        writer.writerow(['timestamp', 'topic', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        
        # Abre o repositório DB3 do ROS2
        with Reader(bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                # Foco apenas nos tópicos de Pose que indicam o caminho do robô
                if connection.topic in ['/odom', '/amcl_pose']:
                    try:
                        # Extrai a mensagem bruta usando RMW CDR nativo
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        
                        # Pegando Pose com Covariância (AMCL/Odom padrão)
                        pose = msg.pose.pose 
                        
                        writer.writerow([
                            timestamp,
                            connection.topic,
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w
                        ])
                    except Exception as e:
                        # Ignorar mensagens malformadas
                        pass
                        
    print(f"Salvo CSV com sucesso: {csv_file}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Exporta Poses de um ROS2 Bag para CSV.")
    parser.add_argument('bag_path', nargs='?', default="--all", help="Caminho para o bag (ou '--all' para converter todos na pasta benchmark_bags/)")
    args = parser.parse_args()
    
    if args.bag_path == '--all':
        import glob
        bags = glob.glob('benchmark_bags/*')
        valid_bags = [b for b in bags if os.path.isdir(b)]
        print(f"Encontrados {len(valid_bags)} bags. Iniciando conversão em lote...")
        for b in valid_bags:
            export_bag_to_csv(b)
    else:
        if not os.path.exists(args.bag_path):
            print(f"Erro: O diretório do bag '{args.bag_path}' não foi encontrado.")
            sys.exit(1)
        export_bag_to_csv(args.bag_path)
