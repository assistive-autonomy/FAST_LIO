#!/usr/bin/env bash
set -euo pipefail
shopt -s extglob

fail() {
  echo >&2 "ERROR: $*"
  exit 1
}

usage() {
  echo "Usage: $0 [--rebuild] [workflow-config.yaml]"
}

rebuild=0
workflow_argument=""
while (( $# > 0 )); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --rebuild)
      [[ $rebuild -eq 0 ]] || fail "--rebuild may only be specified once"
      rebuild=1
      ;;
    --)
      shift
      [[ $# -le 1 ]] || { usage >&2; exit 2; }
      if (( $# == 1 )); then
        workflow_argument="$1"
      fi
      break
      ;;
    -*)
      fail "Unknown option: $1"
      ;;
    *)
      [[ -z "$workflow_argument" ]] || { usage >&2; exit 2; }
      workflow_argument="$1"
      ;;
  esac
  shift
done

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workflow_config="${workflow_argument:-${repo_root}/docker/headless.yaml}"
[[ -f "$workflow_config" ]] || fail "Workflow config not found: $workflow_config"
workflow_config="$(realpath -e -- "$workflow_config")"

declare -A values=()
declare -A seen=()
line_number=0

while IFS= read -r raw_line || [[ -n "$raw_line" ]]; do
  ((line_number += 1))
  raw_line="${raw_line%$'\r'}"
  line="${raw_line##+([[:space:]])}"
  line="${line%%+([[:space:]])}"
  [[ -z "$line" || "$line" == \#* ]] && continue

  if [[ ! "$line" =~ ^([a-zA-Z_][a-zA-Z0-9_]*)[[:space:]]*:[[:space:]]*(.*)$ ]]; then
    fail "${workflow_config}:${line_number}: expected 'key: value'"
  fi

  key="${BASH_REMATCH[1]}"
  value="${BASH_REMATCH[2]}"
  value="${value%%+([[:space:]])}"
  case "$key" in
    input_bag|output_bag|config_file) ;;
    *) fail "${workflow_config}:${line_number}: unknown key '$key'" ;;
  esac
  [[ -z ${seen[$key]+set} ]] || fail "${workflow_config}:${line_number}: duplicate key '$key'"

  if [[ "$value" == \"* ]]; then
    [[ ${#value} -ge 2 && "${value: -1}" == '"' ]] || \
      fail "${workflow_config}:${line_number}: unterminated double-quoted value"
    value="${value:1:${#value}-2}"
    [[ "$value" != *'"'* && "$value" != *'\'* ]] || \
      fail "${workflow_config}:${line_number}: escape sequences are not supported"
  elif [[ "$value" == \'* ]]; then
    [[ ${#value} -ge 2 && "${value: -1}" == "'" ]] || \
      fail "${workflow_config}:${line_number}: unterminated single-quoted value"
    value="${value:1:${#value}-2}"
    [[ "$value" != *"'"* ]] || \
      fail "${workflow_config}:${line_number}: escaped quotes are not supported"
  elif [[ "$value" == *[[:space:]]\#* ]]; then
    value="${value%%+([[:space:]])\#*}"
    value="${value%%+([[:space:]])}"
  fi

  [[ -n "$value" ]] || fail "${workflow_config}:${line_number}: '$key' cannot be empty"
  values[$key]="$value"
  seen[$key]=1
done < "$workflow_config"

for required_key in input_bag output_bag; do
  [[ -n ${values[$required_key]:-} ]] || fail "Missing required key '$required_key' in $workflow_config"
done
config_value="${values[config_file]:-top_autoware_front_imu.yaml}"

input_value="${values[input_bag]}"
output_value="${values[output_bag]}"
[[ "$input_value" == /* ]] || fail "input_bag must be an absolute host path (do not use ~)"
[[ "$output_value" == /* ]] || fail "output_bag must be an absolute host path (do not use ~)"

input_path="$(realpath -e -- "$input_value" 2>/dev/null)" || \
  fail "Input bag does not exist: $input_value"
[[ -f "$input_path" || -d "$input_path" ]] || \
  fail "Input bag must be an MCAP file or rosbag2 directory: $input_path"
[[ -r "$input_path" ]] || fail "Input bag is not readable: $input_path"
[[ "$input_path" != / ]] || fail "Refusing to mount the host root as input_bag"

output_path="$(realpath -m -- "$output_value")"
[[ "$output_path" != / ]] || fail "output_bag cannot be the host root"
[[ ! -e "$output_path" && ! -L "$output_path" ]] || \
  fail "Output bag already exists; choose a new directory: $output_path"
output_name="$(basename -- "$output_path")"
output_parent="$(dirname -- "$output_path")"
[[ "$output_parent" != / ]] || fail "Refusing to mount the host root as the output parent"
mkdir -p -- "$output_parent"
output_parent="$(realpath -e -- "$output_parent")"
[[ -d "$output_parent" && -w "$output_parent" ]] || \
  fail "Output parent is not a writable directory: $output_parent"
output_path="${output_parent}/${output_name}"
[[ "$input_path" != "$output_path" ]] || fail "Input and output bags must be different"
if [[ -d "$input_path" && "$output_path" == "$input_path"/* ]]; then
  fail "output_bag cannot be inside the input bag directory"
fi

if [[ "$config_value" == /* ]]; then
  mapping_config="$config_value"
elif [[ "$config_value" == */* ]]; then
  mapping_config="${repo_root}/${config_value}"
else
  mapping_config="${repo_root}/config/${config_value}"
fi
mapping_config="$(realpath -e -- "$mapping_config" 2>/dev/null)" || \
  fail "FAST-LIO config does not exist: $config_value"
[[ -f "$mapping_config" && -r "$mapping_config" ]] || \
  fail "FAST-LIO config is not a readable file: $mapping_config"

for mount_path in "$input_path" "$output_parent" "$mapping_config"; do
  [[ "$mount_path" != *:* ]] || fail "Docker bind-mount paths cannot contain ':': $mount_path"
done
command -v docker >/dev/null 2>&1 || fail "Docker is not installed"
docker compose version >/dev/null 2>&1 || fail "Docker Compose is not installed"
[[ -f "${repo_root}/include/ikd-Tree/ikd_Tree.cpp" ]] || \
  fail "ikd-Tree is missing; run: git submodule update --init --recursive"

export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

compose=(
  docker compose
  --project-directory "$repo_root"
  --file "${repo_root}/compose.yaml"
)
image_name="$("${compose[@]}" config --images headless)"
[[ -n "$image_name" && "$image_name" != *$'\n'* ]] || \
  fail "Could not resolve exactly one image for the headless service"

echo "Input:  $input_path"
echo "Output: $output_path"
echo "Config: $mapping_config"

if (( rebuild )); then
  echo "Image:  $image_name (rebuild requested)"
  "${compose[@]}" build headless
elif docker image inspect "$image_name" >/dev/null 2>&1; then
  echo "Image:  $image_name (reusing local image)"
else
  echo "Image:  $image_name (not found locally; building once)"
  "${compose[@]}" build headless
fi

exec "${compose[@]}" \
  run --rm --pull never --no-deps --no-TTY \
  --volume "${input_path}:/input/bag:ro" \
  --volume "${output_parent}:/output" \
  --volume "${mapping_config}:/run/fast_lio/config.yaml:ro" \
  headless \
  ros2 run fast_lio fastlio_headless \
    --input /input/bag \
    --output "/output/${output_name}" \
    --config /run/fast_lio/config.yaml
