FROM alessiomorale/jetson-ros-melodic-rtabmap:r32.4.2-cv-4.3.0-py3_0.5.0

# Install rtabmap & prerequisites https://github.com/introlab/rtabmap/blob/master/docker/bionic/Dockerfile
WORKDIR /root/

RUN mkdir -p /ros_melodic_robot/src
COPY ./bootstrap/.rosinstall /ros_melodic_robot/
# build the workspace
WORKDIR /ros_melodic_robot
RUN source /docker-entrypoint.sh && \
    cd /ros_melodic_robot && \
    sed 's/git@github.com:/https:\/\/github.com\//' .rosinstall -i && \
    wstool init -j8 src ./.rosinstall && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys='python3-opencv dkms opencv libopencv-dev libopencv rviz' && \
    rm -rf /ros_melodic_robot/*

# Set up entrypoint
RUN echo "if [ -f /ros_melodic_robot/devel/setup.bash ]; then source /ros_melodic_robot/devel/setup.bash; fi" >> /init_workspaces
