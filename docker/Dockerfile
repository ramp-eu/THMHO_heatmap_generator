FROM ros:noetic

LABEL maintainer "Harshavardhan <harshavardhan.deshpande@ipa.fraunhofer.de>"
SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get -y upgrade

RUN apt-get install -y git \
	&& apt-get -y install python3-pip

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

RUN git clone https://github.com/ipa320/THMHO_heatmap_generator.git

WORKDIR $CATKIN_WS
RUN source /opt/ros/noetic/setup.bash \
	&& rosdep update \
	&& rosdep install --from-paths src --ignore-src -r -y \
	&& catkin_make install \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /entrypoint.sh" >> /root/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
