FROM osrf/ros:humble-desktop-full

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8 

# --- SYSTEM SETUP & DEPENDENCIES ---
# 1. Switch to main Ubuntu mirror
# 2. Enable Universe/Multiverse
# 3. Install ALL dependencies (including the missing libegl1-mesa)
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository multiverse && \
    apt-get update && \
    apt-get install -y \
        # Basic Tools
        ca-certificates gnupg lsb-release sudo wget curl git vim \
        build-essential cmake gdb valgrind rsync \
        # Python Dependencies
        python3-pip python-is-python3 \
        # PX4 & GStreamer Dependencies
        astyle lcov libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav ninja-build protobuf-compiler \
        libeigen3-dev libopencv-dev \
        # VNC, Desktop & VirtualGL Dependencies (FIXED HERE)
        tigervnc-standalone-server xfce4 xfce4-goodies novnc python3-websockify \
        libegl1-mesa libgl1-mesa-glx libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/*

# --- PYTHON PACKAGES ---
RUN pip3 install --no-cache-dir --upgrade pip && \ 
    pip3 install --no-cache-dir \
    empy==3.3.4 pyros-genmsg kconfiglib jsonschema jinja2 \
    pyserial pyyaml packaging toml numpy pandas future lxml cerberus

# --- GAZEBO GARDEN ---
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y gz-garden && \
    rm -rf /var/lib/apt/lists/*

# --- MICRO-XRCE-DDS AGENT ---
RUN cd /tmp && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && make && make install && \
    ldconfig /usr/local/lib/ && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

# --- VIRTUALGL (FIXED) ---
# We use 'apt-get install' instead of 'dpkg -i' to resolve any remaining dependencies automatically
RUN wget https://sourceforge.net/projects/virtualgl/files/3.1/virtualgl_3.1_amd64.deb/download -O /tmp/virtualgl.deb && \
    apt-get update && \
    apt-get install -y /tmp/virtualgl.deb && \
    rm /tmp/virtualgl.deb

# --- USER CONFIGURATION ---
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG dialout $USERNAME

# --- SCRIPTS & WORKSPACE ---
RUN mkdir -p /home/$USERNAME/scripts
COPY .devcontainer/scripts/helper.sh /home/$USERNAME/scripts/helper.sh
COPY .devcontainer/scripts/postCreation.sh /home/$USERNAME/scripts/postCreation.sh
RUN chmod +x /home/$USERNAME/scripts/*.sh && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/scripts

RUN mkdir -p /home/$USERNAME/workspace && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

# --- ENVIRONMENT ---
USER $USERNAME
WORKDIR /home/$USERNAME
ENV PATH="/home/$USERNAME/.local/bin:$PATH"
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/home/$USERNAME/workspace/PX4-Autopilot/Tools/simulation/gz/models" >> ~/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/$USERNAME/workspace/PX4-Autopilot/build/px4_sitl_default/build_gz-garden" >> ~/.bashrc
