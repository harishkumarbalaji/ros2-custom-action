#run a command in the container which is already running using docker-compose
docker exec -it mission_ros2_action-container /bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run mission_processor mission_action_client"
