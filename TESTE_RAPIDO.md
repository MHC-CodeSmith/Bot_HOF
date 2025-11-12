# Teste Rápido - TurtleBot4 no Mundo do Laboratório

## Opção 1: Script Automático (Recomendado)

Execute no terminal:

```bash
./run_lab_world.sh
```

Isso vai:
- Iniciar o contêiner Docker
- Abrir Gazebo com o mundo do laboratório
- Abrir RViz
- Iniciar o TurtleBot4 com SLAM + Nav2

## Opção 2: Manual (Passo a Passo)

### 1. Iniciar o contêiner:

```bash
./run_docker.sh
```

### 2. Dentro do contêiner:

```bash
# Source do ROS
source /opt/ros/humble/setup.bash

# Verificar se o mundo está disponível
ls -la /root/worlds/

# Lançar o TurtleBot4 no mundo do laboratório
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  world:=my_lab \
  rviz:=true \
  slam:=true \
  nav2:=true
```

### 3. Ou usar o script helper:

```bash
./scripts/launch_lab_world.sh
```

## O que esperar:

1. **Gazebo** abre mostrando:
   - Sala 8m x 4m
   - Bancada azul com módulos
   - Equipamentos (esteira verde, prensa vermelha, etc.)
   - TurtleBot4 no centro

2. **RViz** abre mostrando:
   - Visualização do robô
   - Mapas (quando SLAM começar)
   - Navegação (quando Nav2 estiver ativo)

## Comandos úteis:

### Verificar tópicos:
```bash
ros2 topic list
ros2 topic hz /scan
```

### Teleop (controlar o robô):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Salvar mapa:
```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/lab_map
```

## Troubleshooting:

- **Gazebo não abre**: Verifique `DISPLAY` e permissões X11
- **Mundo não carrega**: Verifique `IGN_GAZEBO_RESOURCE_PATH`
- **Robô não aparece**: Verifique os logs: `ros2 topic echo /robot_description`

