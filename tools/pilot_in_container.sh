#!/usr/bin/env bash
# Run go2_turn_pilot.py on the Go2's Jetson Orin inside a NEW throwaway container of the
# existing image go2-slam:nivelado, which already ships unitree_sdk2py 1.0.1 + cyclonedds
# 0.10.2 (RollCall navigation_stack/comandos_stack_nav2.md, sec. 5). Nothing is installed
# on the Orin, and no existing image or container is modified. The container is removed on exit.
#
# Usage (on the Orin, from the repo folder):
#   tools/pilot_in_container.sh --iface eth0 --out data/s1 check
#   tools/pilot_in_container.sh --iface eth0 --out data/s1 sync --tag start
#   tools/pilot_in_container.sh --iface eth0 --out data/s1 run --stick rigid --reps 8
#
# ssh drops: preferably run this inside tmux/screen. If neither exists, use DETACH=1:
#   DETACH=1 tools/pilot_in_container.sh ...   then   docker attach go2_pilot
#   (detach without stopping: Ctrl-P Ctrl-Q; after an ssh drop just ssh back and attach again).
# If the container cannot start as your user, add AS_ROOT=1 (data/ files will then be root-owned).
set -euo pipefail

IMAGE="${IMAGE:-go2-slam:nivelado}"
NAME="go2_pilot"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ $# -eq 0 ]; then
  sed -n "2,15p" "$0"; exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Image $IMAGE not found on this machine. Are you on the Orin?" >&2; exit 1
fi
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "A container named $NAME already exists (another pilot session?)." >&2
  echo "Check with: docker ps -a --filter name=$NAME" >&2; exit 1
fi

# Same two DDS lines RollCall uses for its own SDK bridge (robot_driver.py:107-113):
# load the image's CycloneDDS 0.10.2 (no iceoryx) first, and disable shared memory.
INNER='export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export CYCLONEDDS_URI="<CycloneDDS><Domain><SharedMemory><Enable>false</Enable></SharedMemory></Domain></CycloneDDS>"
exec python3 go2_turn_pilot.py "$@"'

TTY_FLAGS="-it"; [ "${DETACH:-0}" = "1" ] && TTY_FLAGS="-dit"
# Run as the calling user so data/ is not owned by root. If the image refuses that
# (permission errors on start-up), retry with AS_ROOT=1, as RollCall's own docs do.
USER_FLAGS=(--user "$(id -u):$(id -g)" -e HOME=/tmp)
[ "${AS_ROOT:-0}" = "1" ] && USER_FLAGS=()

set -x
docker run --rm $TTY_FLAGS --init --network host --name "$NAME" \
  "${USER_FLAGS[@]}" \
  -v "$REPO_DIR":/pilot -w /pilot \
  "$IMAGE" bash -c "$INNER" pilot "$@"
