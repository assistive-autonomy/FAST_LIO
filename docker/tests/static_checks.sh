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
  docker/headless.yaml
  docker/run_headless.sh
  docker/setup.bash
  docker/tests/container_smoke.sh
)

for path in "${required[@]}"; do
  [[ -f "$path" ]] || { echo >&2 "Missing required file: $path"; exit 1; }
done

[[ -x docker/run_headless.sh ]] || {
  echo >&2 "docker/run_headless.sh is not executable."
  exit 1
}
bash -n docker/run_headless.sh docker/entrypoint.sh docker/setup.bash

[[ -f include/ikd-Tree/ikd_Tree.cpp ]] || {
  echo >&2 "Missing include/ikd-Tree/ikd_Tree.cpp; initialize submodules recursively."
  exit 2
}

expected_ikd_commit=e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4
actual_ikd_commit="$(git -C include/ikd-Tree rev-parse HEAD)"
[[ "$actual_ikd_commit" == "$expected_ikd_commit" ]] || {
  echo >&2 "ikd-Tree is at $actual_ikd_commit; expected $expected_ikd_commit."
  exit 3
}

grep -q '^input_bag:' docker/headless.yaml
grep -q '^output_bag:' docker/headless.yaml
grep -q '^config_file:' docker/headless.yaml
grep -q -- '--input /input/bag' docker/run_headless.sh
grep -q -- '--config /run/fast_lio/config.yaml' docker/run_headless.sh
grep -q -- '--rebuild' docker/run_headless.sh
grep -q 'config --images headless' docker/run_headless.sh
grep -q 'docker image inspect' docker/run_headless.sh
grep -q -- 'run --rm --pull never' docker/run_headless.sh
grep -q 'fastlio_headless' Dockerfile
grep -q '^WORKDIR /work$' Dockerfile
grep -q '^name: fast_lio_headless$' compose.yaml
grep -q 'network_mode: none' compose.yaml
grep -q 'command: \["ros2", "run", "fast_lio", "fastlio_headless", "--help"\]' compose.yaml
grep -q 'git clone --recursive -b headless' docker/README.md

if grep -Eq 'sleep.*infinity|source: ./bag|target: /bag|DISPLAY|/tmp/.X11-unix' compose.yaml; then
  echo >&2 "Interactive GUI settings remain in the headless Compose service."
  exit 4
fi

source_order="$({ grep '^  source ' docker/setup.bash || true; } | sed 's/^  //' | tr '\n' '|')"
expected='source /opt/ros/humble/setup.bash|source /opt/ws_livox/install/setup.bash|source /opt/fast_lio_ws/install/setup.bash|'
[[ "$source_order" == "$expected" ]] || {
  echo >&2 "Environment source order is incorrect: $source_order"
  exit 5
}

if command -v docker >/dev/null 2>&1; then
  docker compose --file compose.yaml config >/dev/null
else
  echo "docker is not installed; skipped Compose validation."
fi

echo "Static Docker checks passed."
