# Wave Rover – ROS 2 Jazzy op de Jetson Orin Nano (JetPack 7.2)
# Basisimage kan in fase 4 vervangen worden door een Isaac ROS-image.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        ros-jazzy-demo-nodes-cpp \
        ros-jazzy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# Werk als gewone gebruiker (uid 1000 = jonas op de Jetson), zodat build/ en
# install/ niet van root worden. Groepen geven toegang tot seriële poorten en camera.
RUN usermod -aG dialout,video ubuntu

COPY docker/entrypoint.sh /entrypoint.sh

USER ubuntu

# Ook voor `docker compose exec ros bash` de ROS-omgeving laden.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/wave_rover/install/setup.bash ] && source ~/wave_rover/install/setup.bash' >> ~/.bashrc
WORKDIR /home/ubuntu/wave_rover

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
