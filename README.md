# rover_launch_files
Launch files for ROS rover

bootstrap folder contains rosinstall and doc to quickly setup a robot.
[Workspace Setup](bootstrap/workspacesetup.md).

Add any needed dependency as runtime dep for rover_launch_files so it is automatically installed by rosdep.

Any dependency from source/repo must be added to `bootstrap/.rosinstall`