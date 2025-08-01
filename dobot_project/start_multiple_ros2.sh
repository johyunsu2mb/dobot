#!/bin/bash

echo "🤖 여러 ROS2 클라이언트 실행"

# 워크스페이스 설정
if [ -d "dobot_workspace" ]; then
    cd dobot_workspace
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash

echo ""
echo "실행 옵션:"
echo "1. 단일 클라이언트 (사용자 이름 입력)"
echo "2. 여러 클라이언트 (자동 이름)"
echo "3. 특정 이름으로 실행"
echo -n "선택 (1-3): "
read choice

case $choice in
    1)
        echo -n "클라이언트 이름 입력: "
        read client_name
        echo "🚀 실행: $client_name"
        ros2 launch dobot_ros2_client dobot_client_named.launch.py client_name:="$client_name"
        ;;
    2)
        echo -n "실행할 클라이언트 수: "
        read num
        for i in $(seq 1 $num); do
            name="ROS2_Node_$i"
            echo "🚀 실행: $name"
            ros2 launch dobot_ros2_client dobot_client_named.launch.py client_name:="$name" &
            sleep 2
        done
        echo ""
        echo "모든 클라이언트가 실행되었습니다."
        echo "종료하려면 Ctrl+C를 누르세요."
        wait
        ;;
    3)
        echo "사용 가능한 이름 예시:"
        echo "  - ROS2_Robot1"
        echo "  - ROS2_Vision"
        echo "  - ROS2_Controller"
        echo "  - ROS2_Monitor"
        echo -n "클라이언트 이름: "
        read custom_name
        ros2 launch dobot_ros2_client dobot_client_named.launch.py client_name:="$custom_name"
        ;;
    *)
        # 기본: 호스트명과 타임스탬프 조합
        default_name="ROS2_$(hostname)_$(date +%s | tail -c 5)"
        echo "🚀 기본 이름으로 실행: $default_name"
        ros2 launch dobot_ros2_client dobot_client_named.launch.py client_name:="$default_name"
        ;;
esac
