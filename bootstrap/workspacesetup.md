# Setup a workspace
Create a folder to host the new workspace

```
mkdir ros_melodic_robot
cd ros_melodic_robot
```
Setup the source folder using wstool:
```
wstool init -j8 src https://github.com/AlessioMorale/rover_launch_files/raw/master/bootstrap/.rosinstall

rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```

Or in a one-liner:

```mkdir ros_melodic_robot && cd ros_melodic_robot && wstool init -j8 src https://github.com/AlessioMorale/rover_launch_files/raw/master/bootstrap/.rosinstall && rosdep install --from-paths src --ignore-src --rosdistro melodic -y```
