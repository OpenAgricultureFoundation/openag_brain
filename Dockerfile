FROM pablogn/rpi-ros-core-indigo

USER pi
# Give the pi user access to the usb drive for flashing an Arduino
RUN sudo usermod -a -G dialout pi
# Install openag_python from the git repository
RUN cd ~ && git clone http://github.com/OpenAgInitiative/openag_python.git
RUN sudo easy_install pip && sudo pip install ./openag_python
RUN sudo pip install pillow --global-option="build_ext" --global-option="--disable-zlib" --global-option="--disable-jpeg"
# Set up a catkin workspace
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && \
    /opt/ros/indigo/env.sh catkin_init_workspace
# Copy in the openag_brain code
ADD . catkin_ws/src/openag_brain
# Install rosserial and openag_brain in the workspace
RUN sudo chown -R pi:pi ~/catkin_ws/src/openag_brain && cd ~/catkin_ws/src && \
    git clone https://github.com/ros-drivers/rosserial.git && \
    cd ~/catkin_ws && /opt/ros/indigo/env.sh catkin_make && \
    sudo apt-get update && \
    sudo apt-get install -y ros-indigo-tf ros-indigo-angles && \
    rosdep update && \
    ~/catkin_ws/devel/env.sh rosdep install -i -y openag_brain && \
    ~/catkin_ws/devel/env.sh rosdep install -i -y rosserial_python && \
    ~/catkin_ws/devel/env.sh rosrun openag_brain install_pio
# Add .bashrc 
RUN echo -e '[ -z "$PS1" ] && return' >~/.bashrc
RUN echo -e 'source ./catkin_ws/devel/setup.bash' >>~/.bashrc
# Run the project
CMD ["~/catkin_ws/devel/env.sh", "rosrun", "openag_brain", "main"]
