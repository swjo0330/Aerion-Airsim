#!/bin/bash
# 모든 시뮬레이션 프로세스 정리
echo "=== 시뮬레이션 프로세스 정리 ==="
pkill -9 UnrealEditor 2>/dev/null && echo "  UnrealEditor 종료" || echo "  UnrealEditor 없음"
pkill -9 -f "bin/px4" 2>/dev/null && echo "  PX4 SITL 종료" || echo "  PX4 없음"
pkill -9 -f "bin/arducopter" 2>/dev/null && echo "  ArduCopter 종료" || echo "  ArduCopter 없음"
pkill -9 -f sim_vehicle 2>/dev/null && echo "  sim_vehicle 종료" || echo "  sim_vehicle 없음"
pkill -9 -f mavproxy 2>/dev/null && echo "  mavproxy 종료" || echo "  mavproxy 없음"
pkill -9 -f bridge_node 2>/dev/null && echo "  ROS2 브릿지 종료" || echo "  ROS2 브릿지 없음"
sleep 2
echo "=== 남은 프로세스 ==="
ps aux | grep -E "UnrealEditor|px4|arducopter|bridge_node" | grep -v grep || echo "  없음 (정상)"
echo "=== GPU 상태 ==="
nvidia-smi --query-gpu=memory.used,temperature.gpu --format=csv,noheader 2>/dev/null
echo "=== 완료 ==="
