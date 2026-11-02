# Build context: PX4-Autopilot/Tools/setup. The source tree is mounted at run
# time, so the image holds only PX4's toolchain and Gazebo.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo lsb-release wget ca-certificates git procps
COPY . /setup/Tools/setup/
RUN cd /setup && bash Tools/setup/ubuntu.sh --no-nuttx && rm -rf /setup
RUN git config --system --add safe.directory '*'
