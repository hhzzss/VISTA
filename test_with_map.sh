#!/usr/bin/env bash
# 与原脚本名兼容，参数原样交给单机启动入口。
set -e

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/test_single_drone.sh" "$@"
