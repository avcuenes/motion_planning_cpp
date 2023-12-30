# Use the official ROS 2 Foxy base image
FROM ros:humble

# Set the working directory
WORKDIR /ros2_workspace

COPY . .
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    libxml2-dev \
    libxslt-dev \
    python3 \
    python3-pip \
    python3-future \
    python3-numpy \
    python3-pytest \
    python3-dev \
    texlive-latex-base \
    doxygen \
    libgtest-dev \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*


# Set environment variables
ENV ROS_DISTRO humble
ENV ROS_VERSION 2

# Source ROS 2 setup script
RUN . /opt/ros/humble/setup.sh && \
    colcon build  

# matplotlib config (used by benchmark)
RUN mkdir -p /root/.config/matplotlib
RUN echo "backend : Agg" > /root/.config/matplotlib/matplotlibrc


# Expose the default ROS 2 communication ports
EXPOSE 11311


# Run a default command, e.g., starting a bash shell
CMD ["bash"]
