#!/bin/zsh

# 设置容器名称
CONTAINER_NAME="data_recording"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WORKSPACE_SETUP="/usr/src/app"

# 参数变量
MODEL_PATH="model.pth"
TOPIC="/suite_1/8/rgb/compressed"
ROSBAG_TOPICS="/ouster/points /suite_1/16/rgb/compressed /suite_1/8/rgb/compressed /suite_2/16/rgb/compressed /suite_2/8/rgb/compressed /vectornav/IMU"

# 修改目录权限
sudo chown -R $(whoami):$(whoami) /media/yyp/My\ Book

# 运行Docker容器
echo ">> Running container $CONTAINER_NAME..."
docker run --gpus all -d --rm --name "$CONTAINER_NAME" --network=host --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /media/yyp/My\ Book/20240806:/usr/rosbags \
    hazardyyp/data_recording:image_classifier \
    bash -c "source $ROS_SETUP && cd $WORKSPACE_SETUP && python3 image_classifier_node.py --model_path $MODEL_PATH --topic $TOPIC --rosbag_path /usr/rosbags --rosbag_topics $ROSBAG_TOPICS"
