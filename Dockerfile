FROM lupasic/jetbot_ros_humble_base:1.0
# Base dockerfile in docker_configs Dockerfile_base. It was built on PC
ARG ROS_VER=humble

USER root
ENV ROS_DISTRO=${ROS_VER}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -q -y \
  libserial-dev \
  python3-numpy \
  python3-opencv && \
  rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_ROOT}/src
RUN git clone https://github.com/ros/xacro.git -b ros2 && \ 
git clone https://github.com/ros-teleop/twist_mux.git -b humble && \
git clone https://github.com/ros-visualization/rqt_robot_steering.git -b 1.0.1 && \
git clone https://github.com/ros-tooling/topic_tools.git -b humble && \
git clone https://github.com/ros2/teleop_twist_joy.git -b humble && \
git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2

WORKDIR ${ROS_ROOT}

RUN INSTALLED=$(ls ${ROS_ROOT}/install/share) && apt update && rosdep install -y \
	               --ignore-src \
	               --from-paths src \
	               --rosdistro ${ROS_DISTRO} \
                   --skip-keys "$(echo $INSTALLED | tr '\n' ' ') twist_mux_msgs"

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 3"

RUN rm -rf ${ROS_ROOT}/src && \
rm -rf ${ROS_ROOT}/log && \
rm -rf ${ROS_ROOT}/build && \
rm -rf /var/lib/apt/lists/*

USER ${APP_USER}

WORKDIR /home/app/ros2_ws