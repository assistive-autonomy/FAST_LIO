# syntax=docker/dockerfile:1.7

ARG ROS_IMAGE=osrf/ros:humble-desktop-full-jammy

FROM ${ROS_IMAGE} AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG LIVOX_SDK2_REPOSITORY=https://github.com/Livox-SDK/Livox-SDK2.git
ARG LIVOX_SDK2_COMMIT=6a940156dd7151c3ab6a52442d86bc83613bd11b
ARG LIVOX_DRIVER_REPOSITORY=https://github.com/Livox-SDK/livox_ros_driver2.git
ARG LIVOX_DRIVER_COMMIT=6b9356cadf77084619ba406e6a0eb41163b08039

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libapr1-dev \
    libaprutil1-dev \
    libeigen3-dev \
    libpcl-dev \
    ninja-build \
    pkg-config \
    python3-colcon-common-extensions \
    python3-dev \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ament-cmake-auto \
    ros-humble-common-interfaces \
    ros-humble-dbw-ford-msgs \
    ros-humble-flir-camera-msgs \
    ros-humble-gps-msgs \
    ros-humble-microstrain-inertial-msgs \
    ros-humble-nav-msgs \
    ros-humble-novatel-gps-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rclcpp-components \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosidl-default-generators \
    ros-humble-rviz2 \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Livox-SDK2 must be installed before livox_ros_driver2 is built.
RUN git clone "${LIVOX_SDK2_REPOSITORY}" /tmp/Livox-SDK2 \
    && git -C /tmp/Livox-SDK2 checkout --detach "${LIVOX_SDK2_COMMIT}" \
    && test "$(git -C /tmp/Livox-SDK2 rev-parse HEAD)" = "${LIVOX_SDK2_COMMIT}" \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/Livox-SDK2/build --parallel "$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && ldconfig \
    && rm -rf /tmp/Livox-SDK2

# livox_ros_driver2 expects to live under <workspace>/src.
RUN mkdir -p /opt/ws_livox/src \
    && git clone "${LIVOX_DRIVER_REPOSITORY}" /opt/ws_livox/src/livox_ros_driver2 \
    && git -C /opt/ws_livox/src/livox_ros_driver2 checkout --detach "${LIVOX_DRIVER_COMMIT}" \
    && test "$(git -C /opt/ws_livox/src/livox_ros_driver2 rev-parse HEAD)" = "${LIVOX_DRIVER_COMMIT}" \
    && source /opt/ros/humble/setup.bash \
    && cd /opt/ws_livox/src/livox_ros_driver2 \
    && ./build.sh humble

WORKDIR /opt/fast_lio_ws/src/FAST_LIO
COPY CMakeLists.txt package.xml ./
COPY config/ config/
COPY include/ include/
COPY launch/ launch/
COPY msg/ msg/
COPY rviz_cfg/ rviz_cfg/
COPY src/ src/

RUN if [[ ! -f include/ikd-Tree/ikd_Tree.cpp ]]; then \
      echo >&2 "ERROR: include/ikd-Tree/ikd_Tree.cpp is missing."; \
      echo >&2 "Clone FAST_LIO with --recursive or run:"; \
      echo >&2 "  git submodule update --init --recursive"; \
      exit 2; \
    fi

RUN source /opt/ros/humble/setup.bash \
    && source /opt/ws_livox/install/setup.bash \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update --rosdistro humble \
    && rosdep install \
         --from-paths /opt/fast_lio_ws/src/FAST_LIO \
         --ignore-src \
         --rosdistro humble \
         --skip-keys livox_ros_driver2 \
         -r -y \
    && cd /opt/fast_lio_ws \
    && colcon build \
         --event-handlers console_direct+ \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && source /opt/fast_lio_ws/install/setup.bash \
    && ros2 pkg prefix fast_lio \
    && ros2 pkg prefix livox_ros_driver2 \
    && ldconfig \
    && ldconfig -p | grep -E 'livox(_lidar)?_sdk' \
    && ! ldd /opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping | grep -q 'not found' \
    && ! ldd /opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_headless | grep -q 'not found'


FROM ${ROS_IMAGE} AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    HOME=/home/${USER_NAME} \
    XDG_RUNTIME_DIR=/tmp/runtime-${USER_NAME} \
    FAST_LIO_WS=/opt/fast_lio_ws \
    FAST_LIO_REPO=/opt/fast_lio_ws/src/FAST_LIO \
    LIVOX_WS=/opt/ws_livox

RUN apt-get update && apt-get install -y --no-install-recommends \
    libapr1 \
    libaprutil1 \
    libpcl-dev \
    ros-humble-dbw-ford-msgs \
    ros-humble-flir-camera-msgs \
    ros-humble-gps-msgs \
    ros-humble-microstrain-inertial-msgs \
    ros-humble-novatel-gps-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/ /usr/local/
COPY --from=builder /opt/ws_livox/install/ /opt/ws_livox/install/
COPY --from=builder /opt/ws_livox/src/livox_ros_driver2/ /opt/ws_livox/src/livox_ros_driver2/
COPY --from=builder /opt/fast_lio_ws/install/ /opt/fast_lio_ws/install/
COPY . /opt/fast_lio_ws/src/FAST_LIO/
RUN install -d -m 0755 /etc/fast_lio
COPY --chmod=0644 docker/setup.bash /etc/fast_lio/setup.bash

RUN ldconfig \
    && if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" "${USER_NAME}"; fi \
    && GROUP_NAME="$(getent group "${USER_GID}" | cut -d: -f1)" \
    && if getent passwd "${USER_NAME}" >/dev/null; then \
         usermod --uid "${USER_UID}" --gid "${GROUP_NAME}" "${USER_NAME}"; \
       else \
         useradd --non-unique --create-home --uid "${USER_UID}" --gid "${GROUP_NAME}" --shell /bin/bash "${USER_NAME}"; \
       fi \
    && HOME_DIR="/home/${USER_NAME}" \
    && mkdir -p \
         /bag \
         /data/Log \
         /data/PCD \
         /input \
         /output \
         /work \
         "${XDG_RUNTIME_DIR}" \
         "${HOME_DIR}" \
    && rm -rf \
         /opt/fast_lio_ws/src/FAST_LIO/Log \
         /opt/fast_lio_ws/src/FAST_LIO/PCD \
    && ln -s /data/Log /opt/fast_lio_ws/src/FAST_LIO/Log \
    && ln -s /data/PCD /opt/fast_lio_ws/src/FAST_LIO/PCD \
    && ln -s /opt/fast_lio_ws "${HOME_DIR}/ros2_ws" \
    && ln -s /opt/ws_livox "${HOME_DIR}/ws_livox" \
    && printf '\nsource /etc/fast_lio/setup.bash\n' >>"${HOME_DIR}/.bashrc" \
    && printf '[[ -f ~/.bashrc ]] && source ~/.bashrc\n' >"${HOME_DIR}/.bash_profile" \
    && chmod 0755 \
         /opt/fast_lio_ws/src/FAST_LIO/docker/*.sh \
         /opt/fast_lio_ws/src/FAST_LIO/docker/tests/*.sh \
    && chown -R "${USER_UID}:${USER_GID}" \
         /data \
         /input \
         /output \
         /work \
         "${XDG_RUNTIME_DIR}" \
         "${HOME_DIR}"

RUN chmod 0700 "${XDG_RUNTIME_DIR}"

RUN source /etc/fast_lio/setup.bash \
    && MAPPING_BIN=/opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_mapping \
    && HEADLESS_BIN=/opt/fast_lio_ws/install/fast_lio/lib/fast_lio/fastlio_headless \
    && DRIVER_LIB=/opt/ws_livox/install/livox_ros_driver2/lib/liblivox_ros_driver2.so \
    && DRIVER_NODE=/opt/ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node \
    && test -x "${MAPPING_BIN}" \
    && test -x "${HEADLESS_BIN}" \
    && test -f "${DRIVER_LIB}" \
    && test -x "${DRIVER_NODE}" \
    && ldd "${MAPPING_BIN}" >/tmp/fast_lio.ldd \
    && ldd "${HEADLESS_BIN}" >/tmp/fast_lio_headless.ldd \
    && ldd "${DRIVER_LIB}" >/tmp/livox_driver_library.ldd \
    && ldd "${DRIVER_NODE}" >/tmp/livox_driver_node.ldd \
    && ! grep -q 'not found' /tmp/fast_lio.ldd \
    && ! grep -q 'not found' /tmp/fast_lio_headless.ldd \
    && ! grep -q 'not found' /tmp/livox_driver_library.ldd \
    && ! grep -q 'not found' /tmp/livox_driver_node.ldd \
    && ldconfig -p | grep -F 'liblivox_lidar_sdk_shared' >/dev/null

COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/fast-lio-entrypoint
COPY --chmod=0755 docker/run_bag.sh /usr/local/bin/fast-lio-play-bag

ENV BASH_ENV=/etc/fast_lio/setup.bash

USER ${USER_NAME}
WORKDIR /work

ENTRYPOINT ["/usr/local/bin/fast-lio-entrypoint"]
CMD ["ros2", "run", "fast_lio", "fastlio_headless", "--help"]
