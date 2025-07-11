FROM autodriveecosystem/autodrive_roboracer_api:2025-icra-practice AS dev

# Hot patch to fix the ros keyrings from the original container
RUN sudo apt-key del F42ED6FBAB17C654 && \
	sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
	sudo rm /etc/apt/sources.list.d/ros2-latest.list

RUN sudo apt-get update -y && sudo apt-get install -y \
    screen \
    psmisc \
    clangd \ 
    clang-format \
    ros-humble-foxglove-bridge \
    nvidia-cuda-toolkit \
    python3-dev \
    python3-pip

# Configure CUDA
ENV CUDA_HOME=/usr
ENV PATH="$CUDA_HOME/bin:${PATH}"
ENV LD_LIBRARY_PATH="$CUDA_HOME/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"

WORKDIR /home/autodrive_devkit
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1"

COPY devkit-startup.bash devkit-startup.bash

EXPOSE 8765
EXPOSE 4567

ENTRYPOINT ["/bin/bash", "/home/autodrive_devkit/devkit-startup.bash"]

FROM dev AS final
WORKDIR /home/autodrive_devkit
COPY ./src/ src/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY devkit-final.bash devkit-final.bash

ENTRYPOINT ["/bin/bash", "/home/autodrive_devkit/devkit-final.bash"]