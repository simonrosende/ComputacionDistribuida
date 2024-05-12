FROM tiryoh/ros-desktop-vnc:noetic
RUN sudo apt update && sudo apt install -y gedit
RUN sudo apt-get install -y libeigen3-dev \
libgtkmm-3.0-dev \
libgstreamermm-1.0-dev \
libyaml-dev

