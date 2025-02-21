FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# Installations for workspace manipulation
RUN apt-get install python3-vcstool python3-colcon-common-extensions git wget -y

# Ssh client allows git management from inside container
RUN apt-get install openssh-client -y

# Ease of use stuff
RUN apt-get install -y nano bash-completion
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo 'alias s="source install/local_setup.bash"' >> /root/.bashrc
# Colcon builds with symlik (so recompile is not needed for python or config changes) and for release
RUN echo 'alias c="colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release"' >> /root/.bashrc
# Suppress colcon specific warning for python package compilation as suggested in https://robotics.stackexchange.com/questions/24230/setuptoolsdeprecationwarning-in-ros2-humble
RUN echo 'PYTHONWARNINGS="ignore:easy_install command is deprecated,ignore:setup.py install is deprecated"; export PYTHONWARNINGS' >> /root/.bashrc
# Select which turtlebot to spawn, see other options in https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/models
ENV TURTLEBOT3_MODEL 'burger'

# Additional ROS stuff
# RUN apt-get install ros-jazzy-rqt-tf-tree -y

# Force color prompt for sudo user
RUN sed -i -e 's/#force_color_prompt=yes/force_color_prompt=yes/g' /root/.bashrc

ENV SHELL /bin/bash

CMD ["/bin/bash"]