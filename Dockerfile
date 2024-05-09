FROM ros:humble

RUN apt update
RUN apt install -y usbutils net-tools software-properties-common wget
RUN apt-get install -y libjpeg-dev libjpeg8-dev libfreetype6-dev vim

RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py

RUN apt-get install -y ros-humble-diagnostic-updater
RUN apt-get install -y ros-humble-tf-transformations
RUN apt install -y ros-humble-navigation2 
RUN apt install -y ros-humble-nav2-bringup 

RUN pip3 install is-msgs==1.1.18 \
    &&   pip3 install is-wire==1.2.1 \
    &&   pip3 install numpy==1.21.6 