# Wave Rover – ROS 2 Jazzy op de Jetson Orin Nano (JetPack 7.2)

# --- llama.cpp met CUDA (module scene: beeldtaalmodel op de GPU) ------------------------
# Apart gebouwd met de CUDA-compiler; in het eindimage komt alleen llama-server (statisch
# gelinkt aan llama.cpp, dynamisch aan CUDA/cuBLAS). Vastgezet op een release zodat het image
# reproduceerbaar is. Compute capability 87 = Orin. Bouwen duurt op de Jetson ±20-40 min;
# -j 3 zodat nvcc (±1 GB per proces) het geheugen niet opgebruikt naast de draaiende robot.
FROM ros:jazzy-ros-base AS llama
ARG DEBIAN_FRONTEND=noninteractive
ARG L4T=r39.2
ARG LLAMA_TAG=b11669
ADD https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/
RUN chmod 644 /etc/apt/trusted.gpg.d/jetson-ota-public.asc \
    && echo "deb https://repo.download.nvidia.com/jetson/common $L4T main" \
        > /etc/apt/sources.list.d/nvidia-jetson.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        git \
        cuda-nvcc-13-2 \
        cuda-cudart-dev-13-2 \
        cuda-driver-dev-13-2 \
        libcublas-dev-13-2 \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 -b "$LLAMA_TAG" https://github.com/ggml-org/llama.cpp.git /opt/llama.cpp \
    && cd /opt/llama.cpp \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
        -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.2/bin/nvcc \
        -DLLAMA_OPENSSL=OFF -DLLAMA_BUILD_TESTS=OFF -DLLAMA_BUILD_EXAMPLES=OFF \
    && cmake --build build --target llama-server -j 3

# --- Wave Rover ------------------------------------------------------------------------
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        alsa-utils \
        python3-serial \
        ros-jazzy-demo-nodes-cpp \
        ros-jazzy-demo-nodes-py \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
        libeigen3-dev \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rosbridge-server \
        ros-jazzy-robot-localization \
        ros-jazzy-slam-toolbox \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-twist-mux \
        ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Pakketten die niet via apt bestaan voor Jazzy, gebouwd uit broncode.
# Vastgezet op een commit zodat het image reproduceerbaar is.
# - sllidar_ros2: Slamtec-driver met C1-ondersteuning (apt-pakket rplidar_ros is te oud)
# - rf2o_laser_odometry: odometrie uit opeenvolgende lidarscans (de motoren hebben geen encoders)
ARG SLLIDAR_COMMIT=34300099fadf
ARG RF2O_COMMIT=b38c68e46387
RUN mkdir -p /opt/thirdparty_ws/src && cd /opt/thirdparty_ws/src \
    && git clone https://github.com/Slamtec/sllidar_ros2.git \
    && git -C sllidar_ros2 checkout "$SLLIDAR_COMMIT" \
    && git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git \
    && git -C rf2o_laser_odometry checkout "$RF2O_COMMIT" \
    && cd /opt/thirdparty_ws && . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# GPU (fase 4): TensorRT en de CUDA-runtime uit de Jetson-repository van NVIDIA. De
# GPU-driver zelf (libcuda) komt bij het starten van de host via CDI (/etc/cdi/nvidia.yaml,
# scripts/install_gpu.sh). L4T moet overeenkomen met de JetPack-versie van de Jetson.
ARG L4T=r39.2
ADD https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/
RUN chmod 644 /etc/apt/trusted.gpg.d/jetson-ota-public.asc \
    && echo "deb https://repo.download.nvidia.com/jetson/common $L4T main" \
        > /etc/apt/sources.list.d/nvidia-jetson.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-13-2 \
        libcublas-13-2 \
        libnvinfer-bin \
        python3-libnvinfer \
        python3-numpy \
        python3-pil \
        ros-jazzy-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Scènebeschrijving (module scene): llama-server uit de eerste stap, en Piper voor de
# Vlaamse stem (losse release met eigen onnxruntime en espeak-ng; stemmen in ~/wave_rover/models).
COPY --from=llama /opt/llama.cpp/build/bin/llama-server /usr/local/bin/llama-server
ARG PIPER_RELEASE=2023.11.14-2
ADD https://github.com/rhasspy/piper/releases/download/${PIPER_RELEASE}/piper_linux_aarch64.tar.gz /tmp/piper.tar.gz
RUN tar -xzf /tmp/piper.tar.gz -C /opt && rm /tmp/piper.tar.gz

# Werk als gewone gebruiker (uid 1000 = jonas op de Jetson), zodat build/ en
# install/ niet van root worden. Groepen geven toegang tot seriële poorten en camera.
RUN usermod -aG dialout,video,audio ubuntu

COPY docker/entrypoint.sh /entrypoint.sh

USER ubuntu

# Ook voor `docker compose exec ros bash` de ROS-omgeving laden.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo 'source /opt/thirdparty_ws/install/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/wave_rover/install/setup.bash ] && source ~/wave_rover/install/setup.bash' >> ~/.bashrc
WORKDIR /home/ubuntu/wave_rover

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
