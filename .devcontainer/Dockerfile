FROM ghcr.io/sloretz/ros:jazzy-desktop

# Configuramos la terminal para evitar problemas interactivos durante la instalación
SHELL [ "/bin/bash" , "-c" ]
ENV DEBIAN_FRONTEND=noninteractive

# 1. Agrupamos apt update e install en un solo paso (Buena práctica de Docker)
# Nota: Como el usuario remoto es root, eliminamos 'sudo'
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    wget \
    terminator \
    git \
    cmake \
    build-essential \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-xacro \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-twist-mux \
    ros-jazzy-robot-localization \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-topic-tools \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalamos los paquetes de Python
RUN pip install --no-cache-dir --break-system-packages \
    pandas \
    numpy \
    scipy \
    icecream \
    pyserial \
    transforms3d

# 3. Agregamos el source de ROS al bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# 4. Clonamos y compilamos CppLinuxSerial
# Eliminamos sudo al final de la compilación
RUN git clone https://github.com/gbmhunter/CppLinuxSerial.git /tmp/CppLinuxSerial \
    && cd /tmp/CppLinuxSerial \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && make -j$(nproc) \
    && make install \
    && cd / \
    && rm -rf /tmp/CppLinuxSerial