# Teste Rápido - TurtleBot4 no Mundo do Laboratório

## Opção 1: Script Automático (Recomendado)

Execute no terminal:

```bash
./run_lab_world.sh
```

Ou com parâmetros customizados:

```bash
# Apenas sim + SLAM (sem Nav2)
./run_lab_world.sh true false

# Com Nav2
./run_lab_world.sh true true

# Com posição inicial customizada
./run_lab_world.sh true true -1.0 0.0 1.57
```

Isso vai:
- Iniciar o contêiner Docker
- Abrir Gazebo (Ignition) com o mundo do laboratório
- Abrir RViz
- Iniciar o TurtleBot4 com SLAM + Nav2 (se habilitado)

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
./scripts/launch_lab_world.sh true true 0.0 0.0 0.0
```

Ou manualmente:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  world:=my_lab \
  rviz:=true \
  slam:=true \
  nav2:=true \
  use_sim_time:=true
```

## O que esperar:

1. **Gazebo (Ignition)** abre mostrando:
   - Sala 8.7m x 5.3m (dimensões reais)
   - Bancada Festo/Siemens (2.27m x 0.8m) com módulos
   - Equipamentos (esteira verde, prensa vermelha, elevador, sensor)
   - PLCs e HMIs na estrutura vertical
   - Pilar cilíndrico branco
   - TurtleBot4 no centro (ou posição especificada)

2. **RViz** abre mostrando:
   - Visualização do robô
   - Mapas (quando SLAM começar)
   - Navegação (quando Nav2 estiver ativo)

## Comandos úteis:

### Verificar tópicos:
```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom
```

### Verificar TF:
```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

### Diagnóstico completo:
```bash
./scripts/check_tf_scan.sh
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
- **Mundo não carrega**: Verifique `IGN_GAZEBO_RESOURCE_PATH` e se `worlds/my_lab.sdf` existe
- **Robô não aparece**: Verifique os logs: `ros2 topic echo /robot_description`
- **Nav2 não inicia**: Veja `NAV2_FIX.md` para diagnóstico detalhado
- **TF não funciona**: Execute `./scripts/check_tf_scan.sh` para diagnóstico

## Parâmetros do script:

```bash
./run_lab_world.sh [SLAM] [NAV2] [X] [Y] [YAW]
```

- `SLAM`: `true` ou `false` (padrão: `true`)
- `NAV2`: `true` ou `false` (padrão: `true`)
- `X`: Posição X inicial em metros (padrão: `0.0`)
- `Y`: Posição Y inicial em metros (padrão: `0.0`)
- `YAW`: Orientação inicial em radianos (padrão: `0.0`)

## Exemplos:

```bash
# Apenas simulação (sem SLAM, sem Nav2)
./run_lab_world.sh false false

# SLAM sem Nav2 (recomendado para testar primeiro)
./run_lab_world.sh true false

# Tudo ativado
./run_lab_world.sh true true

# Posição inicial customizada (x=-1.0, y=0.0, yaw=90°)
./run_lab_world.sh true true -1.0 0.0 1.57
```
