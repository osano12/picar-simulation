FROM osrf/ros:humble-desktop-full

# Éviter les interactions avec l'utilisateur pendant l'installation
ENV DEBIAN_FRONTEND=noninteractive

# Installer les dépendances
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-matplotlib \
    python3-yaml \
    python3-flask \
    python3-websocket \
    python3-requests \
    python3-psutil \
    python3-pytest \
    git \
    curl \
    wget \
    vim \
    nano \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Installer les dépendances Python
RUN pip3 install --no-cache-dir \
    flask-socketio \
    eventlet \
    pytest-ros \
    jupyterlab \
    ipywidgets \
    opencv-python \
    transforms3d \
    pyyaml \
    rospkg

# Créer un utilisateur non-root
RUN useradd -m -d /home/ros -s /bin/bash ros \
    && echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

# Définir l'utilisateur
USER ros
WORKDIR /home/ros

# Créer l'espace de travail ROS 2
RUN mkdir -p /home/ros/ros2_ws/src

# Copier les fichiers du projet
COPY --chown=ros:ros . /home/ros/ros2_ws/src/picar_simulator/

# Construire l'espace de travail
WORKDIR /home/ros/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Configurer l'environnement
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc \
    && echo "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc

# Exposer les ports
EXPOSE 8080 9090

# Point d'entrée
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/ros/ros2_ws/install/setup.bash && ros2 launch picar_simulator picar_simulator.launch.py"]