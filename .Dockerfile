# ROSPlan docker image
FROM ros:melodic

SHELL ["/bin/bash", "-c"]
WORKDIR /root/ws


# install dependencies
RUN apt update -qq && apt install python-catkin-tools git -y -qq

# Create WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    mkdir src &&\
    catkin init &&\
    catkin build --summarize  --no-status

# Copy source files. 
COPY . ./src/rosplan

# ROSPlan demos are cloned from the related repository
RUN git clone --depth 1 https://github.com/clearpathrobotics/occupancy_grid_utils.git src/occupancy_grid_utils &&\
    git clone --depth 1 https://github.com/KCL-Planning/rosplan_demos.git src/rosplan_demos

# Get ROSPlan from repo
#RUN git clone --recurse-submodules --shallow-submodules --depth 1 https://github.com/KCL-Planning/ROSPlan.git src/rosplan &&\
#   git clone --depth 1 https://github.com/clearpathrobotics/occupancy_grid_utils.git src/occupancy_grid_utils &&\
#   git clone --depth 1 https://github.com/KCL-Planning/rosplan_demos.git src/rosplan_demos


# Further dependencies
RUN source devel/setup.bash &&\
    rosdep update &&\
    rosdep install --from-paths src/ --ignore-src -q -r -y

# Build workspace
RUN catkin build --summarize --no-status

# Prepare workspace for runtime. Set the prompt to be colored
RUN echo -e "source /opt/ros/$ROS_DISTRO/setup.bash\nsource devel/setup.bash" >> ~/.bashrc &&\
    sed -i s/^#force_color_prompt/force_color_prompt/g ~/.bashrc