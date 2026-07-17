#!/usr/bin/env bash
set -eo pipefail

source /etc/fast_lio/setup.bash

mkdir -p /data/Log /data/PCD

exec "$@"
