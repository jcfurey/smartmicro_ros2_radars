# Build/test image for the smartmicro ROS 2 workspace.
#   docker build --build-arg ROS_DISTRO=jazzy --build-arg USER_UID=$(id -u) -t umrr-ros:jazzy .
# The source tree is bind-mounted at /code at run time (see docker-compose.yml);
# USER_UID should match the owner of that checkout so builds can write to it.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO
ARG USERNAME=builder
ARG USER_UID=1000

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Keep images small: never pull recommended packages (also applies to rosdep's apt calls).
RUN echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/99-no-recommends \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        iputils-ping \
        python3-pip \
        wget \
    && rm -rf /var/lib/apt/lists/*

# Install every workspace dependency declared in the package manifests.
COPY umrr_ros2_msgs/package.xml /tmp/deps/umrr_ros2_msgs/package.xml
COPY umrr_ros2_driver/package.xml /tmp/deps/umrr_ros2_driver/package.xml
COPY smart_rviz_plugin/package.xml /tmp/deps/smart_rviz_plugin/package.xml
COPY smartmicro_description/package.xml /tmp/deps/smartmicro_description/package.xml
COPY smartmicro_processing/package.xml /tmp/deps/smartmicro_processing/package.xml
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /tmp/deps --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/* /tmp/deps

# Unprivileged build user with the host checkout's UID (reuse the image's existing
# user for that UID, e.g. "ubuntu" on Noble-based images).
RUN if getent passwd "${USER_UID}" >/dev/null; then \
        usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m "$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
    else \
        useradd -m -u "${USER_UID}" -s /bin/bash "${USERNAME}"; \
    fi \
    && mkdir -p /code \
    && chown "${USER_UID}" /code

USER ${USERNAME}
WORKDIR /code
