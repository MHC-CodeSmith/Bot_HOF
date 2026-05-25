# Navegação TurtleBot4 - Estabilização Nav2

## Status Final da Sessão
- [x] **Relógios:** Sincronizados (< 5ms de drift).
- [x] **Cadeia de Comando:** Redirecionada para `/cmd_vel_unstamped` (sem conflitos de TwistStamped).
- [x] **Ajuste Fino:** Paredes finas (0.24m) e tolerância de TF otimizada (1.0s AMCL, 2.5s servidores).
- [x] **Planejamento:** Plano de integração com MyCobot via `map -> mycobot_base_link` aprovado.

## Resumo Técnico (Baseline)
- **Arquivo de Params:** [nav2_normal_fino.yaml](file:///home/mhc/Germany/turtlebot4_jazzy_docker/config/nav2_normal_fino.yaml)
- **Transform Tolerance:** AMCL (1.0s) / Outros (2.5s)
- **Tópico Velocity:** `/cmd_vel_unstamped`

## Comando para Iniciar Manualmente
```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/mhc/Germany/turtlebot4_jazzy_docker/maps/mapa_quarto_final.yaml \
  params_file:=/home/mhc/Germany/turtlebot4_jazzy_docker/config/nav2_normal_fino.yaml \
  use_sim_time:=False
```
