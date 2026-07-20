#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

required=(
  Dockerfile
  .dockerignore
  compose.yaml
  .env.example
  docker/README.md
  docker/entrypoint.sh
  docker/run_bag.sh
  docker/setup.bash
  docker/tests/container_smoke.sh
  docker/tests/validate_bag.sh
)

for path in "${required[@]}"; do
  [[ -f "$path" ]] || { echo >&2 "Missing required file: $path"; exit 1; }
done

[[ -f include/ikd-Tree/ikd_Tree.cpp ]] || {
  echo >&2 "Missing include/ikd-Tree/ikd_Tree.cpp; clone/update submodules recursively."
  exit 2
}

expected_ikd_commit=e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
if git -C include/ikd-Tree rev-parse HEAD >/dev/null 2>&1; then
  actual_ikd_commit="$(git -C include/ikd-Tree rev-parse HEAD)"
  [[ "$actual_ikd_commit" == "$expected_ikd_commit" ]] || {
    echo >&2 "ikd-Tree is at $actual_ikd_commit; expected $expected_ikd_commit."
    exit 5
  }
else
  echo >&2 "Could not verify the ikd-Tree commit from Git metadata."
  exit 5
fi

grep -q '6a940156dd7151c3ab6a52442d86bc83613bd11b' Dockerfile
grep -q '6b9356cadf77084619ba406e6a0eb41163b08039' Dockerfile
grep -q 'osrf/ros:humble-desktop-full-jammy' Dockerfile
grep -q 'fastlio_playback_qos.yaml' docker/run_bag.sh
grep -q 'jazzy_mcap_fastlio_qos.yaml' docker/run_bag.sh

source_order="$({ grep '^  source ' docker/setup.bash || true; } | sed 's/^  //' | tr '\n' '|')"
expected='source /opt/ros/humble/setup.bash|source /opt/ws_livox/install/setup.bash|source /opt/fast_lio_ws/install/setup.bash|'
[[ "$source_order" == "$expected" ]] || {
  echo >&2 "Environment source order is incorrect: $source_order"
  exit 3
}

grep -q 'source /etc/fast_lio/setup.bash' docker/entrypoint.sh
grep -q 'source /etc/fast_lio/setup.bash' Dockerfile
grep -q 'ln -s /opt/fast_lio_ws.*ros2_ws' Dockerfile
grep -q '^WORKDIR /bag$' Dockerfile
grep -q 'command: \["sleep", "infinity"\]' compose.yaml
grep -q 'source: ./bag' compose.yaml
grep -q 'target: /bag' compose.yaml

if grep -Eq 'BAG_DIR|OUTPUT_DIR|/bags|\./bags' \
  compose.yaml docker/README.md Dockerfile .env.example; then
  echo >&2 "Legacy configurable bag/output mount paths found."
  exit 6
fi

if grep -Eqi 'lio[-_ ]?sam|gtsam' \
  Dockerfile compose.yaml docker/entrypoint.sh docker/run_bag.sh \
  docker/setup.bash docker/tests/container_smoke.sh docker/tests/validate_bag.sh; then
  echo >&2 "Unexpected LIO-SAM/GTSAM dependency found."
  exit 4
fi

if command -v docker >/dev/null 2>&1; then
  docker compose config >/dev/null
else
  echo "docker is not installed; skipped 'docker compose config'."
fi

echo "Static Docker checks passed."
