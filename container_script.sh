echo "Running container"

docker run -it \
    --rm \
    --volume="/home/matheus/task_manager/ros2_ws:/ros2_ws" \
    --net=host \
    --pid=host \
    --ipc=host \
    --name=task_manager_dev \
    -t task_manager_ros2:0.0.1 \
    bash
echo "Done."