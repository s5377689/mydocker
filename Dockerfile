FROM dsalvat1/cudagl:12.3.1-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# —— 在 apt 裝其他東西之前就放這段 ——  （升級 CMake）
RUN apt-get update -y && apt-get install -y wget gnupg lsb-release && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc \
        | gpg --dearmor -o /usr/share/keyrings/kitware-archive.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive.gpg] \
        https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/kitware.list && \
    apt-get update -y && \
    apt-get install -y cmake && \
    rm -rf /var/lib/apt/lists/*

# 安裝 apt其他相關套件
RUN apt-get update && apt-get install -y git libxcb-xinerama0 libxcb-xinerama0-dev libreadline-dev \
    libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 \
    gedit sudo\
    && echo 'export QT_X11_NO_MITSHM=1' >> /home/$USERNAME/.bashrc \
    && echo 'export QT_XCB_GL_INTEGRATION=xcb_glx' >> /home/$USERNAME/.bashrc \
    && rm -rf /var/lib/apt/lists/*

# 加入 universe
RUN apt-get update \
    && apt-get install -y software-properties-common \
    && add-apt-repository universe

# 設定 ROS 2 軟體庫和金鑰
RUN apt-get update \
    && apt-get install -y curl lsb-release \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb

# 安裝 ROS2 Humble、其他相關套件
RUN apt-get update \
    && apt-get install -y \
        ros-humble-desktop \
        python3-colcon-common-extensions python3-pexpect \
        python3-pip libsqlite3-dev nlohmann-json3-dev \
        default-jre \
    && rm -rf /var/lib/apt/lists/*

# 安裝 gz-harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    # && apt-get install -y gz-harmonic ros-humble-ros-gz-sim ros-humble-xacro ros-humble-ros-gz-bridge
    && apt-get install -y gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge ros-humble-xacro
####################################################################################################
# 創建使用者
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN mkdir -p /etc/sudoers.d \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# 建立 ROS2 工作空間、下載 ardupilot ros2.repos
RUN apt-get update && apt-get install -y python3-vcstool python3-rosdep git \
    && mkdir -p /home/$USERNAME/install/src \
    && cd /home/$USERNAME/install \
    # && mkdir -p /home/$USERNAME/ros2_ws/src \
    # && cd /home/$USERNAME/ros2_ws \
    && vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src \
    && rosdep init \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y

# **這一行 chown 要放在 user 建立好之後！**
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME
####################################################################################################
# Set up ROS 2 workspace
USER $USERNAME
WORKDIR /home/$USERNAME
ENV PATH="${PATH}:/home/${USERNAME}/.local/bin"
# Clone Micro-XRCE-DDS-Gen 並編譯
RUN cd /home/$USERNAME/install \
    && git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
    && cd Micro-XRCE-DDS-Gen \
    && ./gradlew assemble
ENV PATH="/home/$USERNAME/install/Micro-XRCE-DDS-Gen/scripts:${PATH}"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/$USERNAME/install && colcon build --packages-up-to ardupilot_dds_tests"

# Clone BehaviorTree.CPP 並編譯
RUN pip3 install --no-cache-dir conan
RUN conan profile detect --force
RUN cd /home/$USERNAME/install && \
    git clone --depth 1 https://github.com/BehaviorTree/BehaviorTree.CPP.git && \
    cd BehaviorTree.CPP && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_SQLITE_LOGGER=OFF \
             -DBUILD_ZMQ_PUBLISHER=OFF \
             -DBUILD_EXAMPLES=OFF && \
    cmake --build . -j$(nproc) && \
    cmake --install . --prefix /home/$USERNAME/.local

#安裝 ardupilot_gazebo
USER root

RUN apt-get update && apt-get install -y \
        libgz-sim8-dev rapidjson-dev \
        libopencv-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    # ------- 立刻修補舊版 jsoncpp 的 CMake policy，其他行不動 -------
    && for f in /usr/lib/x86_64-linux-gnu/cmake/jsoncpp/jsoncppConfig.cmake \
                /usr/share/cmake/gz-cmake3/cmake3/FindJSONCPP.cmake ; do \
           sed -i -E 's/cmake_policy\(VERSION [0-9\.]+\)/cmake_policy(VERSION 3.5)/g' "$f" ; \
       done \
    && rm -rf /var/lib/apt/lists/*

COPY --chown=user:user install /home/$USERNAME/install

USER $USERNAME

RUN cd /home/$USERNAME/install && \
    # git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
    cd ardupilot_gazebo && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j$(nproc) && \
    make install DESTDIR=/home/$USERNAME/.local && \
    cp -r /home/$USERNAME/.local/usr/local/* /home/$USERNAME/.local/

# 安裝浮力波浪插件 gz_maritime_ws
USER root

RUN apt-get update && apt-get install -y python3-pip wget unzip \
    && pip3 install -U colcon-common-extensions \
    && wget https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/tutorials/files/surface_vehicles/gz_maritime_ws.zip \
           -O /home/$USERNAME/install/gz_maritime_ws.zip \
    && unzip /home/$USERNAME/install/gz_maritime_ws.zip -d /home/$USERNAME/install \
    && rm  /home/$USERNAME/install/gz_maritime_ws.zip \
    && for f in /usr/lib/x86_64-linux-gnu/cmake/jsoncpp/jsoncppConfig.cmake \
                /usr/share/cmake/gz-cmake3/cmake3/FindJSONCPP.cmake ; do \
           sed -i -E 's/cmake_policy\(VERSION [0-9\.]+\)/cmake_policy(VERSION 3.5)/g' "$f" ; \
       done \
    && cd /home/$USERNAME/install/gazebo_maritime_ws && colcon build --merge-install \
    && rm -rf /var/lib/apt/lists/*

# 複製 Qt6.8.3 到 /opt/qt6.8.3
COPY --chown=user:user 6.8.3/gcc_64 /home/$USERNAME/install/qt6.8.3
ENV PATH="/home/$USERNAME/install/qt6.8.3/bin:$PATH"
# ENV CMAKE_PREFIX_PATH="/home/$USERNAME/install/qt6.8.3"
ENV CMAKE_PREFIX_PATH="/home/$USERNAME/install/qt6.8.3:/home/$USERNAME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

# 複製 ros2_ws 到容器中
COPY --chown=user:user ros2_ws /home/$USERNAME/ros2_ws

RUN chown -R $USERNAME:$USERNAME /home/$USERNAME
USER $USERNAME

# --- Gazebo plugins & models -------------------------------------------------
# ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/$USERNAME/install/ardupilot_gazebo/build:/home/$USERNAME/install/gazebo_maritime_ws/install/lib
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/home/$USERNAME/.local/lib/ardupilot_gazebo:/home/$USERNAME/install/gazebo_maritime_ws/install/lib
# ENV GZ_SIM_RESOURCE_PATH=/home/$USERNAME/install/ardupilot_gazebo/models:/home/$USERNAME/install/ardupilot_gazebo/worlds:/home/$USERNAME/install/gazebo_maritime_ws/install/share/gazebo_maritime/models
ENV GZ_SIM_RESOURCE_PATH=/home/$USERNAME/.local/share/ardupilot_gazebo/models:/home/$USERNAME/install/ardupilot_gazebo/worlds:/home/$USERNAME/install/gazebo_maritime_ws/install/share/gazebo_maritime/models
# ENV LD_LIBRARY_PATH=/home/$USERNAME/install/ardupilot_gazebo/build:/home/$USERNAME/install/gazebo_maritime_ws/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
ENV LD_LIBRARY_PATH=/home/$USERNAME/.local/lib:/home/$USERNAME/install/ardupilot_gazebo/build:/home/$USERNAME/install/gazebo_maritime_ws/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
# 安裝 ArduPilot 主專案
RUN sudo apt-get update && sudo apt-get install -y python3-future libfuse2
RUN pip install MAVProxy
RUN cd /home/$USERNAME/install && \
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    git submodule update --init --recursive&& \
    # --------- 新增兩行：更新到修掉 JSON segfault 的 commit 再編譯 Rover ----------
    ./waf configure --board sitl && ./waf rover -j$(nproc)
ENV PATH="/home/$USERNAME/install/ardupilot/Tools/autotest:${PATH}"

# # 複製 ros2_ws 到容器中
# COPY --chown=user:user ros2_ws /home/$USERNAME/ros2_ws

WORKDIR /home/$USERNAME/ros2_ws
RUN rm -rf build install log
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
#                  && source /home/$USERNAME/install/install/setup.bash \
#                  && colcon build --symlink-install --packages-select ardupilot_msgs custom_msgs"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash  \
                #  && source /home/$USERNAME/ros2_ws/install/setup.bash \
                 && source /home/$USERNAME/install/install/setup.bash \
                 && colcon build --symlink-install"


# 自動 source ROS2 環境
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/install/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'export ROS_DOMAIN_ID=0' >> /home/$USERNAME/.bashrc \
    && echo 'export RMW_FASTRTPS_USE_SHM=OFF' >> /home/$USERNAME/.bashrc

# 下載 QGroundControl AppImage
WORKDIR /home/$USERNAME/Downloads
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage -O QGroundControl.AppImage && \
    chmod +x QGroundControl.AppImage

WORKDIR /home/$USERNAME/ros2_ws
CMD ["bash"]