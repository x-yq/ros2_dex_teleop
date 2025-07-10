#!/bin/bash
# Print environment details
echo "Path to log: $SRC/log/"

DEV=${DEV:-true}

if [ -d $WS ]; then
    rm -r $WS
fi 

echo "Create ROS WORK SPACE"
mkdir -p $WS_SRC

#if symbolic link is already created remove it 
if [ -L "$WS_SRC" ]; then
    rm "$WS_SRC"
fi    

if [ "$DEV" = "true" ]; then 
    echo "Development mode enabled: Creating symlinks for each ros2 pkg to ros workspace src" 
    ln -s $SRC/data_collection/ $WS_SRC
    ln -s $SRC/interfaces/ $WS_SRC
    ln -s $SRC/deployment/ $WS_SRC

else 
    echo "Development mode disabled: Copying source files to workspace $WS_SRC"
    cp -r $SRC/data_collection/ $WS_SRC
    cp -r $SRC/interfaces/ $WS_SRC
    cp -r $SRC/deployment/ $WS_SRC
fi

## TODO: setup third-party repo here
cd $WS_SRC

# Source ROS 2 environment
if [ -d "/opt/ros/humble/install" ]; then
    source /opt/ros/humble/install/setup.bash
else
    source /opt/ros/humble/setup.bash
fi
if ! grep -qF "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "Adding ros humbel setup sourcing to ~/.bashrc"
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

cd $WS 
colcon build

# Source the workspace setup file
echo "Sourcing workspace setup file..."
source "${WS}/install/setup.bash"
if ! source "${WS}/install/setup.bash"; then
    echo "Failed to source workspace setup file. Exiting."
    exit 1
fi
if ! grep -qF "source ${WS}/install/setup.bash" ~/.bashrc; then
    echo "Adding workspace sourcing to ~/.bashrc"
    echo "source ${WS}/install/setup.bash" >> ~/.bashrc
fi

## Launch the runtime with the specified parameters
# e.g. ros2 launch xarm_moveit_config xarm5_moveit_gazebo.launch.py
/bin/bash