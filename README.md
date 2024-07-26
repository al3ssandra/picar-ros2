# picar-ros2

picar-ros2 is a project implementing a simulation of the Picar 4WD using ros2

## Docker instructions

To build the docker image for this project run the following commands starting from the root of the workspace (this repo is the workspace, so in this case is the folder picar-ros2):

```bash
cd .devcontainer
docker build
```

After building the container check the image ID by running
```bash
docker images
```

To allow the container to display the rviz and Gazebo windows on your local computer, run the following command replacing IMAGE_ID with your image ID
```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' IMAGE_ID`
```

Finally, you can run the container with the following command, replacing PATH_TO_WS and IMAGE_ID with the right values:
```bash
docker run -d --rm --network host -e DISPLAY=$DISPLAY -v PATH_TO_WS:/home/picar_ws IMAGE_ID
```

Check the container ID with the following command:
```bash
docker ps
```

You can acces the terminal inside the container with the following command, replacing CONTAINER_ID with the right value:
```bash
docker exec -it CONTAINER_ID bash
```

If your using Visual Studio Code and you have installed the Docker extension, you can instead right click on the container and select Attatch Visual Studio Code

## Usage

Inside the container run:
```bash
sudo apt update
sudo apt upgrade
```

Then run the following command from the root of the workspace (in this case it's in /home/picar_ws) to install the ros2 dependencies (ros2 packages needed)
```bash
rosdep update 
rosdep install --from-paths src --ignore-src -y
```

Finally source the ros2 setup file, with the following command
```bash
source /opt/ros/iron/setup.bash
```

Or if you don't want to have to source it everytime you open a new terminal, then use the following command and restart the terminal
```bash
echo 'source /opt/ros/iron/setup.bash' >> ~/.bashrc
```

Finally to run the simulation in gazebo and visualization in rviz run:
```bash
colcon build
source install/setup.bash
source /usr/share/gazebo/setup.sh
ros2 launch carlikebot_nav sensors.launch.py
```

After Rviz and Gazebo launch, click in ```2D Pose Estimate``` in Rviz and then click on the map such that the arrow corresponds to the orientation of the car in Gazebo. After that you can click on ```2D Goal Pose``` to move the car where you want it to and you should see the car moving in Rviz and Gazebo.

## Visual Studio Code

If you have installed the Docker and Dev Containers extensions in Visual Studio Code, you can press F1 from the root of the workspace and that will open the command palette, from wich you can select  ```Dev Containers: Reopen in Container```, and that will automatically build and run the container.

![](screencast.webm)