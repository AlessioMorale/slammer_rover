FROM alessiomorale/jetson-ros-rtabmap:melodic_r32.4.4_cv4.4.0_1.2
# Install rtabmap & prerequisites https://github.com/introlab/rtabmap/blob/master/docker/bionic/Dockerfile
WORKDIR /root/

RUN mkdir -p /ros_melodic_robot/src
COPY ./bootstrap/.rosinstall /ros_melodic_robot/
# build the workspace
WORKDIR /ros_melodic_robot

RUN source /init_workspaces && \
    cd /ros_melodic_robot && \
    sed 's/git@github.com:/https:\/\/github.com\//' .rosinstall -i && \
    wstool init -j8 src ./.rosinstall && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys='python3-opencv dkms opencv libopencv-dev cv-bridge libopencv rviz' && \
    pip3 --no-cache install pyserial && \
    rm -rf /ros_melodic_robot/*

ENV ROS_MASTER_URI http://127.0.0.1:11311/
RUN echo "source /init_workspaces" > /root/.bashrc

RUN mkdir -p /root/ccache
RUN echo 'export "PATH=/usr/lib/ccache:$PATH"' >> /root/.bashrc

# Set up entrypoint
RUN echo "if [ -f /ros_melodic_robot/devel/setup.bash ]; then source /ros_melodic_robot/devel/setup.bash; fi" >> /init_workspaces
RUN echo "if [ -f /ros_melodic_robot/devel/setup.bash ]; then source /ros_melodic_robot/devel/setup.bash; fi" >> /init_workspaces

ARG STARTUP_PACKAGE=slammer_bringup
ENV STARTUP_PACKAGE=${STARTUP_PACKAGE}
ARG STARTUP_LAUNCH=bringup.launch
ENV STARTUP_LAUNCH=${STARTUP_LAUNCH}
CMD [ "/bin/bash", "-c", "source /init_workspaces && roslaunch ${STARTUP_PACKAGE} ${STARTUP_LAUNCH} --wait"]