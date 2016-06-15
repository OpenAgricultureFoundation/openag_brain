FROM pablogn/rpi-ros-core-indigo

# Set up a catkin workspace
USER pi
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && \
    /opt/ros/indigo/env.sh catkin_init_workspace
ADD . catkin_ws/src/openag_brain
RUN sudo chown -R pi:pi ~/catkin_ws/src/openag_brain && cd ~/catkin_ws/src && \
    git clone https://github.com/ros-drivers/rosserial.git && \
    cd ~/catkin_ws && /opt/ros/indigo/env.sh catkin_make && \
    sudo apt-get update && \
    sudo apt-get install -y python-pip ros-indigo-tf ros-indigo-angles && \
    rosdep update && \
    ~/catkin_ws/devel/env.sh rosdep install -i -y openag_brain && \
    ~/catkin_ws/devel/env.sh rosrun openag_brain install_pio

CMD ["~/catkin_ws/devel/env.sh", "rosrun", "openag_brain", "main"]
