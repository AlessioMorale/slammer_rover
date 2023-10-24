# syntax = docker/dockerfile:1.2
FROM alessiomorale/jetson-ros-rtabmap:melodic_r32.4.4_cv4.4.0_2.0.1
RUN rm -f /etc/apt/apt.conf.d/docker-clean
# Install rtabmap & prerequisites https://github.com/introlab/rtabmap/blob/master/docker/bionic/Dockerfile
WORKDIR /root/

RUN mkdir -p /ros_melodic_robot/src
COPY /slammer_rover/bootstrap/.rosinstall /ros_melodic_robot/
# build the workspace
WORKDIR /ros_melodic_robot
COPY * /ros_melodic_robot/src
RUN source /init_workspaces && \
    cd /ros_melodic_robot && \
    sed 's/git@github.com:/https:\/\/github.com\//' .rosinstall -i
#    && wstool init -j8 src ./.rosinstall --shallow 
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys='python3-opencv python-opencv dkms opencv libopencv-dev cv-bridge libopencv rviz gazebo_ros gazebo_ros_control' && \
    pip3 --no-cache install pyserial && \
    rm -rf /ros_melodic_robot/*
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get install --no-install-recommends software-properties-common -y \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install python3.8-dev python3.8 python3.8-venv -y \
    && python3.8 --version
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get install net-tools -y --no-install-recommends && pip3 install wheel && pip3 install Adafruit_SSD1306
ENV ROS_MASTER_URI http://127.0.0.1:11311/
RUN echo "source /init_workspaces" > /root/.bashrc
RUN apt install net-tools libusb-dev -y --no-install-recommends

RUN mkdir -p /root/ccache
RUN echo 'export "PATH=/usr/lib/ccache:$PATH"' >> /root/.bashrc

RUN ldconfig

# Set up entrypoint
RUN echo "if [ -f /ros_melodic_robot/devel/setup.bash ]; then source /ros_melodic_robot/devel/setup.bash; fi" >> /init_workspaces

ARG ROSCONSOLE_FORMAT="'[\${node} | \${severity} | \${time}]: \${message}'"
ENV ROSCONSOLE_FORMAT=$ROSCONSOLE_FORMAT
CMD [ "/bin/bash", "-c", "source /init_workspaces && roslaunch ${STARTUP_PACKAGE} ${STARTUP_LAUNCH} --wait"]
