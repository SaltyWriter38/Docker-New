FROM osrf/ros:humble-desktop-full

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8 

# 1. Install Basic Dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates gnupg lsb-release sudo wget curl git vim \
    build-essential cmake python3-pip python-is-python3 \
    gdb valgrind \
    && rm -rf /var/lib/apt/lists/*

# 2. Configure Git for Windows Compatibility (CRITICAL)
RUN git config --system core.autocrlf false && \
    git config --system core.eol lf

# 3. Install Python Tools (Fixed Typos)
RUN pip3 install --no-cache-dir --upgrade pip && \ 
    pip3 install --no-cache-dir \
    empy==3.3.4 pyros-genmsg kconfiglib jsonschema jinja2 \
    pyserial pyyaml packaging toml numpy pandas future lxml cerberus

# 4. Install Gazebo Garden (Fixed wget typo)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y gz-garden && \
    rm -rf /var/lib/apt/lists/*

# 5. Install PX4 Dependencies (Fixed GStreamer typo)
RUN apt-get update && apt-get install -y \
    astyle lcov libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav ninja-build protobuf-compiler \
    libeigen3-dev libopencv-dev rsync \
    && rm -rf /var/lib/apt/lists/*

# 6. Install Micro-XRCE-DDS Agent
RUN cd /tmp && \
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && make && make install && \
    ldconfig /usr/local/lib/ && \
    cd / && rm -rf /tmp/Micro-XRCE-DDS-Agent

# 7. Create User and Fix Permissions
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG dialout $USERNAME

# 8. Setup Scripts (Fixed Recursive Ownership)
RUN mkdir -p /home/$USERNAME/scripts
COPY helper.sh /home/$USERNAME/scripts/helper.sh
COPY postCreation.sh /home/$USERNAME/scripts/postCreation.sh
RUN chmod +x /home/$USERNAME/scripts/*.sh && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/scripts

# 9. Setup Workspace
RUN mkdir -p /home/$USERNAME/workspace && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# 10. Fix PATH and Bash Environment
ENV PATH="/home/$USERNAME/.local/bin:$PATH"
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/home/$USERNAME/workspace/PX4-Autopilot/Tools/simulation/gz/models" >> ~/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/$USERNAME/workspace/PX4-Autopilot/build/px4_sitl_default/build_gz-garden" >> ~/.bashrc
