# Use a imagem do Ubuntu 20.04 como base
FROM ubuntu:20.04

#resettin all docker system: docker system prune -a --volumes
# Configuração inicial do ambiente
ENV DEBIAN_FRONTEND=noninteractive

# Instalar pacotes do sistema
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    lsb-release \
    software-properties-common \
    curl \
    build-essential \
    cmake \
    git   \
    python3-tk \
    build-essential

# Adicionar o repositório ROS Noetic
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Instalar o ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Inicializar o ambiente ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Criar workspace do ROS
RUN mkdir -p /root/catkin_ws/src
RUN mkdir -p /root/shared
WORKDIR /root/shared/


# Instalar o VS Code
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libx11-xcb1 \
        libxcb-dri3-0 \
        libdrm2 \
        libgbm1 \
        libasound2 \
        libatspi2.0-0 \
        libgtk-3-0 \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.edge.kernel.org/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu|http://mirrors.edge.kernel.org/ubuntu/|g' /etc/apt/sources.list


RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    libignition-math4-dev \
    ros-noetic-geographic-msgs

#go to src directory
RUN cd /root/catkin_ws/src

# Adicionar a chave SSH e configurar o ambiente SSH
#https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
#https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account
#copiar sua chave:
#cp /home/"user name"/.ssh/id_ed25519 "path/to/dockerfile"

COPY id_ed25519 /root/.ssh/id_ed25519
RUN chmod 600 /root/.ssh/id_ed25519 && \
    ssh-keyscan github.com >> /root/.ssh/known_hosts

#migbot:
RUN git clone -b ros-noetic-devel git@github.com:ttrindader/migbot_gazebo.git /root/catkin_ws/src/migbot_gazebo

#plugins for https://github.com/ttrindader/migbot_gazebo/tree/ros-noetic-devel
# Clone the mig_gazebo_plugins repository
RUN git clone git@github.com:ttrindader/mig_gazebo_plugins.git /root/catkin_ws/src/mig_gazebo_plugins


# Build the mig_gazebo_plugins
WORKDIR /root/catkin_ws/src/mig_gazebo_plugins
RUN mkdir build && cd build && cmake .. && make -j4 && make install


#go to src livox laser
RUN cd /root/catkin_ws/src
RUN git clone git@github.com:Livox-SDK/livox_laser_simulation.git /root/catkin_ws/src/livox_laser_simulation
RUN cd /root/catkin_ws/src/livox_laser_simulation
WORKDIR /root/catkin_ws/src/livox_laser_simulation

# Update CMakeLists.txt to use C++17 instead of C++11
RUN sed -i 's/-std=c++11/-std=c++17/gi' CMakeLists.txt

RUN echo 'source /usr/share/gazebo-11/setup.bash' >> ~/.bashrc \
    echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/root/catkin_ws/src/migbot_gazebo/migbot_gazebo/models' >> ~/.bashrc


#go to src directory
RUN cd /root/catkin_ws/src

RUN git clone -b ros-noetic-devel git@github.com:ttrindader/migenv_gazebo.git /root/catkin_ws/src/migenv_gazebo
RUN echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/root/catkin_ws/src/migenv_gazebo/mig_gazebo/models' >> ~/.bashrc
RUN . /root/.bashrc
RUN cd /root/catkin_ws/src

# Clonar o pacote desvio_obstaculo para a pasta src
#RUN git clone https://github.com/ruffeny/desvio_obstaculo.git /root/catkin_ws/src/desvio_obstaculo

#go back to catkin_ws workspace
#RUN cd /root/catkin_ws/

#git clone -b ros-noetic-devel https://github.com/ttrindader/migenv_gazebo

# Definir o ponto de entrada padrão para o shell
CMD ["bash"]    

RUN apt-get update
RUN apt-get install -y python3-tk


RUN echo 'export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/"your_workspace"/src/migenv_gazebo/mig_gazebo/models' >> ~/.bashrc

# Source ROS and catkin_make
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"

# Set up the environment and build the catkin workspace with C++14
WORKDIR /root/catkin_ws/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_CXX_FLAGS='-std=c++17'"

RUN mkdir /home/shared
WORKDIR /root/catkin_ws/
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
# RUN source devel/setup.bash


#xhost +local:root
#docker build -t ruffeny:husky .
#docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" ruffeny:husky



