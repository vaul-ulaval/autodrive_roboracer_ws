FROM autodriveecosystem/autodrive_f1tenth_api:2024-cdc-practice AS dev

RUN sudo apt-get update -y && sudo apt-get install -y \
    screen \
    psmisc \
    clangd \ 
    clang-format \
    ros-humble-foxglove-bridge \
    ros-humble-laser-filters \
    ros-humble-robot-localization \ 
    ros-humble-ackermann-msgs \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-topic-tools \
    nvidia-cuda-toolkit \
    python3-dev \
    python3-pip

# Configure CUDA
ENV CUDA_HOME=/usr
ENV PATH="$CUDA_HOME/bin:${PATH}"
ENV LD_LIBRARY_PATH="$CUDA_HOME/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"

COPY devkit-startup.bash devkit-startup.bash

EXPOSE 8765
EXPOSE 4567

ENTRYPOINT ["/bin/bash", "devkit-startup.bash"]

FROM dev AS final
COPY ./src/ /home/autodrive_devkit/src/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY devkit-startup.bash devkit-startup.bash

ENTRYPOINT ["/bin/bash", "final-startup.bash"]