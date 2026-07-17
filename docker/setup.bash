#!/usr/bin/env bash

if [[ "${FAST_LIO_SETUP_SOURCED:-0}" != "1" ]]; then
  source /opt/ros/humble/setup.bash
  source /opt/ws_livox/install/setup.bash
  source /opt/fast_lio_ws/install/setup.bash

  export FAST_LIO_WS=/opt/fast_lio_ws
  export FAST_LIO_REPO=/opt/fast_lio_ws/src/FAST_LIO
  export LIVOX_WS=/opt/ws_livox
  export LD_LIBRARY_PATH="/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export FAST_LIO_SETUP_SOURCED=1
fi
