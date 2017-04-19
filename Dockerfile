FROM resin/rpi-raspbian
USER root
# Install python and some barebones tools
# (Things you would typically have in the Pi's environment)
RUN apt-get update && \
    apt-get install -y -q \
    wget \
    git \
    vim \
    locales \
    build-essential \
    python \
    libpython-dev \
    libboost-chrono-dev \
    libboost-date-time-dev \
    libboost-program-options-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libtinyxml-dev \
    libboost-filesystem-dev \
    libxml2-dev \
    libgtest-dev
# Tell apt to read from the ROS package repository
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
# Create pi user
RUN useradd pi && echo 'pi:hypriot' | chpasswd && echo "pi ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && mkdir -p /home/pi && chown pi:pi /home/pi

USER pi
WORKDIR /home/pi/
# Give the pi user access to the usb drive for flashing an Arduino
RUN sudo usermod -a -G dialout pi
# Add catkin install dir (built with catkin)
# Note you must run `docker build` from `install` dir so the build context
# includes the install directory. Typically this is done like so:
#
#    cd ~/catkin_ws
#    docker build -f ./src/openag_brain/Dockerfile .
ADD install /home/pi/catkin_ws/install/
RUN sudo chown -R pi:pi ~/catkin_ws
# Install ROS boostrapping tool
RUN sudo apt-get update && sudo apt-get install --no-install-recommends -y -q \
    python-pip python-rosdep
# Install dependencies with rosdep
RUN sudo rosdep init && rosdep update && rosdep install --from-paths ~/catkin_ws/install --ignore-src --rosdistro indigo -y -r --os=debian:jessie
# Install openag_python from the git repository
RUN cd ~ && git clone http://github.com/OpenAgInitiative/openag_python.git
RUN sudo pip install ./openag_python
RUN sudo locale-gen en_US.UTF-8
# Add .bashrc
RUN echo -e '[ -z "$PS1" ] && return' >~/.bashrc
RUN echo -e 'source ~/catkin_ws/install/setup.bash' >>~/.bashrc

# Set up ROS environment vars
ENV LANG=en_US.UTF-8 ROS_DISTRO=indigo
# Run the project
CMD ["/home/pi/catkin_ws/install/env.sh", "rosrun", "openag_brain", "main", "personal_food_computer_v2.launch"]
USER pi
