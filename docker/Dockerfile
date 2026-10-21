# Wave Rover – ROS 2 Jazzy op de Jetson Orin Nano (JetPack 7.2)
# Basisimage kan in fase 4 vervangen worden door een Isaac ROS-image.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        ros-jazzy-demo-nodes-cpp \
        ros-jazzy-demo-nodes-py \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-twist-mux \
        ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Slamtec-driver met ondersteuning voor de RPLidar C1 (het apt-pakket rplidar_ros
# gebruikt een te oude SDK). Vastgezet op een commit zodat het image reproduceerbaar is.
ARG SLLIDAR_COMMIT=34300099fadf
RUN mkdir -p /opt/sllidar_ws/src && cd /opt/sllidar_ws/src \
    && git clone https://github.com/Slamtec/sllidar_ros2.git \
    && git -C sllidar_ros2 checkout "$SLLIDAR_COMMIT" \
    && cd /opt/sllidar_ws && . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Werk als gewone gebruiker (uid 1000 = jonas op de Jetson), zodat build/ en
# install/ niet van root worden. Groepen geven toegang tot seriële poorten en camera.
RUN usermod -aG dialout,video ubuntu

COPY docker/entrypoint.sh /entrypoint.sh

USER ubuntu

# Ook voor `docker compose exec ros bash` de ROS-omgeving laden.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo 'source /opt/sllidar_ws/install/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/wave_rover/install/setup.bash ] && source ~/wave_rover/install/setup.bash' >> ~/.bashrc
WORKDIR /home/ubuntu/wave_rover

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
