# Correção do Nav2 - Guia de Uso

Este guia explica como usar o Nav2 corrigido com os parâmetros customizados.

## 🔧 O que foi corrigido

1. **RMW Implementation**: Fixado `rmw_fastrtps_cpp` (padrão no Jazzy e compatível com Create3 firmware I.*)
2. **use_sim_time**: Garantido `true` em todos os nós
3. **Parâmetros do Nav2**: Arquivo customizado com frames e costmaps corretos
4. **Scripts de diagnóstico**: Ferramentas para verificar TF e scan

## 📁 Arquivos criados

- `params/tb4_nav2_params.yaml` - Parâmetros customizados do Nav2
- `scripts/check_tf_scan.sh` - Script de diagnóstico
- `scripts/launch_nav2_separate.sh` - Launch do Nav2 separado

## 🚀 Como usar

### Opção 1: Tudo junto (recomendado para teste)

```bash
./run_lab_world.sh true true 0.0 0.0 0.0
```

Isso inicia:
- Simulação + SLAM + Nav2 + RViz
- Usa os parâmetros customizados automaticamente

### Opção 2: Passo a passo (recomendado se Nav2 falhar)

#### Passo 1: Iniciar apenas sim + SLAM

```bash
./run_lab_world.sh true false 0.0 0.0 0.0
```

Aguarde até:
- ✅ Gazebo abrir
- ✅ RViz abrir
- ✅ LaserScan publicando (`/scan`)
- ✅ TF estável (`odom -> base_link`)

#### Passo 2: Verificar TF e scan

Em outro terminal (dentro do contêiner):

```bash
./scripts/check_tf_scan.sh
```

Deve mostrar:
- ✓ Tópico `/scan` encontrado
- ✓ LaserScan publicando mensagens
- ✓ Transform `odom -> base_link` OK

#### Passo 3: Iniciar Nav2 separadamente

Em outro terminal (dentro do contêiner):

```bash
./scripts/launch_nav2_separate.sh
```

Ou manualmente:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Se turtlebot4_navigation estiver disponível:
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true

# Ou usando nav2_bringup:
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true
```

## 🔍 Diagnóstico

### Verificar tópicos

```bash
ros2 topic list | grep -E '/scan$|/odom$|/cmd_vel$'
```

### Verificar TF

```bash
# Deve funcionar sem timeout
ros2 run tf2_ros tf2_echo odom base_link

# Se SLAM estiver rodando:
ros2 run tf2_ros tf2_echo map base_link
```

### Verificar parâmetros do costmap

```bash
ros2 param dump /global_costmap
ros2 param dump /local_costmap
```

Verifique se:
- `global_frame: map`
- `robot_base_frame: base_link`
- `odom_frame: odom`
- `use_sim_time: True`

### Ver logs do Nav2

```bash
ros2 run rqt_console rqt_console
```

Filtre por:
- `planner_server`
- `global_costmap`
- `controller_server`

## ⚠️ Problemas comuns

### 1. "Couldn't initialize state machine for node global_costmap"

**Causa**: TF ausente ou frames incorretos

**Solução**:
1. Verifique TF: `ros2 run tf2_ros tf2_echo odom base_link`
2. Aguarde SLAM estabilizar (se usando SLAM)
3. Verifique frames nos parâmetros

### 2. "Waiting for service controller_server/get_state"

**Causa**: Planner ou controller não iniciaram

**Solução**:
1. Verifique logs: `ros2 run rqt_console rqt_console`
2. Inicie Nav2 separadamente após sim estar estável
3. Verifique se `use_sim_time:=true` está em todos os nós

### 3. LaserScan não aparece

**Causa**: Tópico ou frame incorreto

**Solução**:
```bash
# Verificar tópico
ros2 topic echo -n 1 /scan | grep frame_id

# Verificar se frame é transformável
ros2 run tf2_ros tf2_echo base_link <frame_do_scan>
```

## 📝 Parâmetros importantes

No arquivo `params/tb4_nav2_params.yaml`:

- **global_costmap**:
  - `global_frame: map`
  - `robot_base_frame: base_link`
  - `topic: /scan`

- **local_costmap**:
  - `global_frame: odom`
  - `robot_base_frame: base_link`
  - `topic: /scan`

- **Todos os nós**:
  - `use_sim_time: True`

## 🎯 Próximos passos

1. Teste com sim + SLAM primeiro
2. Verifique TF e scan
3. Inicie Nav2 separadamente
4. Se funcionar, tente tudo junto

## 📚 Referências

- [Nav2 Documentation](https://navigation.ros.org/)
- [TurtleBot4 Documentation](https://turtlebot.github.io/turtlebot4-docs/)

