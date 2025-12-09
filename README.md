# TurtleBot4 Simulator (ROS 2 Jazzy + Gazebo Harmonic) — Docker

Ambiente **reprodutível** para simular o TurtleBot4 com **Gazebo (Harmonic)**,
usar **SLAM (slam_toolbox)**, **salvar mapa** e depois rodar **localização + Nav2**.

> Testado em Ubuntu 24.04 com GPU NVIDIA (OpenGL 4.6) via `--gpus all`.

## Estrutura

```
turtlebot4_docker/
├─ Dockerfile
├─ run_docker.sh
├─ maps/                # mapas salvos (yaml/pgm)
├─ scripts/             # (opcional) helper scripts
├─ notes/ .dev/         # itens locais (não versionar)
└─ README.md
```

## Requisitos

- Docker e NVIDIA Container Toolkit (para GPU NVIDIA)
- X11 ativo (para RViz2 e GUI do Gazebo)
- Internet (baixar mundos e pacotes Gazebo/ROS)

## Build

```bash
docker build --no-cache -t turtlebot4:jazzy .
```

## Rodar o contêiner (com GPU e X11)

```bash
chmod +x run_docker.sh
./run_docker.sh
```

Dentro do contêiner:

```bash
source /opt/ros/jazzy/setup.bash
```

## 1) Simulação com SLAM + Nav2 + RViz

```bash
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py \
  slam:=true nav2:=true rviz:=true
```

* Em RViz:

  * `Fixed Frame = map`
  * Adicione **LaserScan** (`/scan`) e **Map** (`/map`) se necessário.
* Verifique:

  ```bash
  ros2 topic hz /scan
  ros2 topic echo /map --once --qos-durability transient_local
  ```
* Movimente o robô (teleop integrado no painel da GUI).

### Salvar o mapa

Se o contêiner foi iniciado com `-v $PWD/maps:/root/maps`:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/warehouse
# Gera maps/warehouse.yaml e maps/warehouse.pgm no host
```

## 2) Rodar com **Localização + Nav2** (sem SLAM)

Feche o SLAM e rode:

```bash
# só a simulação (se preferir rodar separado)
# ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py

# Localização (AMCL) usando o mapa salvo
ros2 launch turtlebot4_navigation localization.launch.py map:=/root/maps/warehouse.yaml

# Nav2
ros2 launch turtlebot4_navigation nav2.launch.py
```

Em RViz:

* Use **2D Pose Estimate** para inicializar a pose.
* Envie metas com **2D Goal Pose** ou pelo painel *Navigation2*.

### Checks úteis

```bash
ros2 topic list | grep map
ros2 topic info /map -v
ros2 node list | grep amcl
ros2 run tf2_tools view_frames  # gera frames.pdf
```

## Solução de problemas

* **TF `map→odom→base_link` quebrado**: abra `view_frames` e confirme que há um único grafo conectado.
* **Map não aparece**: confira QoS `TRANSIENT_LOCAL` em `/map`:
  `ros2 topic echo /map --once --qos-durability transient_local`
* **GUI travando**: confirme aceleração de GPU no contêiner:

  ```bash
  apt-get update && apt-get install -y mesa-utils
  glxinfo | grep -E "OpenGL renderer|OpenGL version"
  ```

## Licença

MIT (ou escolha a sua).
