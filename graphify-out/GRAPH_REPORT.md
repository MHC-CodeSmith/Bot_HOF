# Graph Report - turtlebot4_jazzy_docker  (2026-05-11)

## Corpus Check
- 21 files · ~495,074 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 240 nodes · 289 edges · 19 communities (17 shown, 2 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 9 edges (avg confidence: 0.6)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `74570a62`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]

## God Nodes (most connected - your core abstractions)
1. `MissionManager` - 17 edges
2. `MissionBase` - 17 edges
3. `LidColorClassifierSimple` - 16 edges
4. `MissionManager` - 13 edges
5. `DeliveryRoutine` - 11 edges
6. `LidColorClassifier` - 9 edges
7. `RestockRoutine` - 9 edges
8. `FailureRoutine` - 9 edges
9. `Correção do Nav2 - Guia de Uso` - 9 edges
10. `Teste Rápido - TurtleBot4 no Mundo do Laboratório` - 8 edges

## Surprising Connections (you probably didn't know these)
- `DeliveryRoutine` --uses--> `MissionBase`  [INFERRED]
  scripts/delivery_routine.py → scripts/mission_base.py
- `RestockRoutine` --uses--> `MissionBase`  [INFERRED]
  scripts/restock_routine.py → scripts/mission_base.py
- `FailureRoutine` --uses--> `MissionBase`  [INFERRED]
  scripts/failure_routine.py → scripts/mission_base.py
- `MissionManager` --uses--> `DeliveryRoutine`  [INFERRED]
  scripts/mission_manager.py → scripts/delivery_routine.py
- `MissionManager` --uses--> `FailureRoutine`  [INFERRED]
  scripts/mission_manager.py → scripts/failure_routine.py

## Communities (19 total, 2 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.09
Nodes (8): MissionBase, DeliveryRoutine, FailureRoutine, main(), MissionManager, Nó gerenciador de missões.     Recebe triggers via service e despacha para a rot, Verifica periodicamente se o Nav2 está disponível., RestockRoutine

### Community 1 - "Community 1"
Cohesion: 0.12
Nodes (10): MissionBase, Envia goal de navegação para o waypoint `wp_name`.          `done_cb(True)`  → c, Executa undock. Pula se o robô já estiver fora da dock., Executa dock com retentativas., Navega para predock_point e depois chama dock., Sinaliza ao mission_manager que esta rotina acabou., Converte yaw 2D para componentes z, w do quaternion., Classe base compartilhada por todas as rotinas de missão.     Declara parâmetros (+2 more)

### Community 2 - "Community 2"
Cohesion: 0.09
Nodes (21): 1. SLAM + Nav2 + RViz (Mapping), 2. Saving the Map, 3. Localization + Nav2 (Navigation), Build, code:text (turtlebot4_docker/), code:bash (ros2 topic echo /map --once --qos-durability transient_local), code:bash (apt-get update && apt-get install -y mesa-utils), code:bash (docker build --no-cache -t turtlebot4:jazzy .) (+13 more)

### Community 3 - "Community 3"
Cohesion: 0.1
Nodes (20): 1. "Couldn't initialize state machine for node global_costmap", 2. "Waiting for service controller_server/get_state", 3. LaserScan não aparece, 📁 Arquivos criados, code:bash (# Verificar tópico), code:bash (ros2 topic list | grep -E '/scan$|/odom$|/cmd_vel$'), code:bash (# Deve funcionar sem timeout), code:bash (ros2 param dump /global_costmap) (+12 more)

### Community 4 - "Community 4"
Cohesion: 0.16
Nodes (5): main(), MissionManager, 2D yaw -> quaternion (z,w)., Waypoint, yaw_to_quat_z_w()

### Community 5 - "Community 5"
Cohesion: 0.12
Nodes (16): 1. Iniciar o contêiner:, 2. Dentro do contêiner:, code:bash (./run_lab_world.sh), code:bash (./run_lab_world.sh [SLAM] [NAV2] [X] [Y] [YAW]), code:bash (# Apenas simulação (sem SLAM, sem Nav2)), code:bash (# Apenas sim + SLAM (sem Nav2)), code:bash (./run_docker.sh), code:bash (# Source do ROS) (+8 more)

### Community 6 - "Community 6"
Cohesion: 0.23
Nodes (3): LidColorClassifierSimple, main(), Versão simples:     - usa ROI central da imagem     - ignora a faixa inferior (t

### Community 8 - "Community 8"
Cohesion: 0.21
Nodes (5): main(), NetworkTester, Node, main(), UndockClient

### Community 9 - "Community 9"
Cohesion: 0.17
Nodes (11): 1. Raw IP Communication (ICMP Ping), 2. ROS 2 Transport Instrumentation (Fast DDS RTT & Drop Rate), code:text ([RX] Reply seq=2 | RTT=14.27 ms | Avg RTT=14.27 ms | Loss Ra), code:text ([TX] msg seq=1 | size=1000056 bytes), Conclusion, Methodology, Network Communication Drill-Down Analysis, Recommended Fixes (+3 more)

### Community 10 - "Community 10"
Cohesion: 0.29
Nodes (5): _cleanup(), LidColorClassifier, main(), Subscribes:  /camera/color/image_raw  (sensor_msgs/Image)     Publishes:   /prod, Filtra a cor, acha contornos e aproxima polígonos. Retorna (mask, poly).

### Community 11 - "Community 11"
Cohesion: 0.18
Nodes (11): code:bash (./run_lab_world.sh true true 0.0 0.0 0.0), code:bash (./run_lab_world.sh true false 0.0 0.0 0.0), code:bash (./scripts/check_tf_scan.sh), code:bash (./scripts/launch_nav2_separate.sh), code:bash (source /opt/ros/jazzy/setup.bash), 🚀 Como usar, Opção 1: Tudo junto (recomendado para teste), Opção 2: Passo a passo (recomendado se Nav2 falhar) (+3 more)

### Community 12 - "Community 12"
Cohesion: 0.18
Nodes (11): code:bash (ros2 run nav2_map_server map_saver_cli -f /root/maps/lab_map), code:bash (ros2 topic list), code:bash (ros2 run tf2_ros tf2_echo odom base_link), code:bash (./scripts/check_tf_scan.sh), code:bash (ros2 run teleop_twist_keyboard teleop_twist_keyboard), Comandos úteis:, Diagnóstico completo:, Salvar mapa: (+3 more)

### Community 13 - "Community 13"
Cohesion: 0.43
Nodes (6): main(), opcua_connect_test(), parse_host_port(), Testa se dá pra abrir conexão TCP host:port., Testa handshake OPC UA com user/senha., tcp_probe()

## Knowledge Gaps
- **65 isolated node(s):** `Testa se dá pra abrir conexão TCP host:port.`, `Testa handshake OPC UA com user/senha.`, `Nó gerenciador de missões.     Recebe triggers via service e despacha para a rot`, `Verifica periodicamente se o Nav2 está disponível.`, `Subscribes:  /camera/color/image_raw  (sensor_msgs/Image)     Publishes:   /prod` (+60 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MissionBase` connect `Community 1` to `Community 8`, `Community 0`?**
  _High betweenness centrality (0.096) - this node is a cross-community bridge._
- **Why does `MissionManager` connect `Community 4` to `Community 8`?**
  _High betweenness centrality (0.067) - this node is a cross-community bridge._
- **Why does `MissionManager` connect `Community 0` to `Community 8`?**
  _High betweenness centrality (0.059) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `MissionBase` (e.g. with `DeliveryRoutine` and `RestockRoutine`) actually correct?**
  _`MissionBase` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `MissionManager` (e.g. with `DeliveryRoutine` and `FailureRoutine`) actually correct?**
  _`MissionManager` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `DeliveryRoutine` (e.g. with `MissionManager` and `MissionBase`) actually correct?**
  _`DeliveryRoutine` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Testa se dá pra abrir conexão TCP host:port.`, `Testa handshake OPC UA com user/senha.`, `Nó gerenciador de missões.     Recebe triggers via service e despacha para a rot` to the rest of the system?**
  _65 weakly-connected nodes found - possible documentation gaps or missing edges._