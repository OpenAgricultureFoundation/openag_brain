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

RUN sudo locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create pi user
RUN useradd pi && echo 'pi:hypriot' | chpasswd && echo "pi ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && mkdir -p /home/pi && chown pi:pi /home/pi

# Give pi access to the camera device
RUN sudo usermod -a -G video pi

RUN groupadd gpio

USER pi
WORKDIR /home/pi/

# We use COPY rather than ADD, as COPY is preferred for local files.
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/#add-or-copy
COPY . /home/pi/catkin_ws/src/openag_brain
RUN sudo chown -R pi:pi ~/catkin_ws

RUN /home/pi/catkin_ws/src/openag_brain/scripts/install_dev

# Run the project
CMD ["/home/pi/catkin_ws/install/env.sh", "rosrun", "openag_brain", "main", "personal_food_computer_v2.launch"]
USER pi
