# rhaschke/ici:rviz-noetic-ros
# Image used for GHA CI builds
# build with:
# - DOCKER_BUILDKIT=1 docker build --tag rhaschke/ici:rviz-noetic-ros - < .github/workflows/Dockerfile
# - DOCKER_BUILDKIT=1 docker build --tag rhaschke/ici:rviz-jammy-ros --build-arg="BASE=ubiagni/ros:jammy-ros-base" --build-arg="ROS_DISTRO=one" - < .github/workflows/Dockerfile

ARG ROS_DISTRO=noetic
ARG BASE=ros:noetic-ros-base
FROM ${BASE}

ENV TERM xterm

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=bind,target=/tmp/rviz \
    # Update apt package list as previous containers clear the cache
    apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -q install --no-install-recommends -y \
        # Some basic requirements
        wget git sudo \
        # Qt6 packages for Jammy
        $(test "$ROS_DISTRO" = "one" && echo "qt6-base-dev qt6-base-dev-tools libqt6opengl6-dev") \
        # Preferred build tools
        clang clang-format-12 clang-tidy clang-tools ccache && \
    #
    # Download all dependencies for rviz
    rosdep update $(test "${ROS_DISTRO}" = "melodic" && echo "--include-eol-distros") && \
    DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths /tmp/rviz --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/* && \
    # Globally disable git security
    # https://github.blog/2022-04-12-git-security-vulnerability-announced
    git config --global --add safe.directory "*"
