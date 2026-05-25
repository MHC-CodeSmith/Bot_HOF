#!/bin/bash
echo "--- Lifecycle Status ---"
for node in controller_server planner_server behavior_server bt_navigator smoother_server velocity_smoother collision_monitor docking_server amcl map_server; do
    echo -n "$node: "
    ros2 lifecycle get /$node 2>/dev/null || echo "Not found"
done

echo ""
echo "--- TF Tree map -> base_link ---"
ros2 run tf2_ros tf2_echo map base_link --once
