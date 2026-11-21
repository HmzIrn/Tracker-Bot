#!/usr/bin/env bash
set -eo pipefail

usage() {
    printf 'Usage: %s [--no-build|--remove]\n' "${0##*/}" >&2
}

fail() {
    printf 'register_with_ros.sh: %s\n' "$1" >&2
    exit 1
}

case "$#" in
    0) mode="register" ;;
    1)
        case "$1" in
            --no-build) mode="no-build" ;;
            --remove) mode="remove" ;;
            *) usage; exit 2 ;;
        esac
        ;;
    *) usage; exit 2 ;;
esac

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace_root="$(cd -- "$script_dir/.." && pwd -P)"
bashrc="$HOME/.bashrc"
registration_file="$HOME/.survellence_sim.bash"
backup_file="$HOME/.bashrc.survellence_sim.bak"
start_marker="# >>> Survellence_Sim >>>"
end_marker="# <<< Survellence_Sim <<<"

rewrite_bashrc() {
    local action="$1"
    local temporary

    if [[ "$action" == "install" && ! -f "$bashrc" ]]; then
        : > "$bashrc"
    fi
    if [[ ! -f "$bashrc" ]]; then
        return 0
    fi

    temporary="$(mktemp "$HOME/.bashrc.survellence_sim.XXXXXX")"
    if ! python3 - "$bashrc" "$temporary" "$action" "$start_marker" "$end_marker" <<'PY'
import sys
from pathlib import Path

source_path, output_path, action, start_marker, end_marker = sys.argv[1:]
raw = Path(source_path).read_bytes()
lines = raw.splitlines(keepends=True)
start = start_marker.encode()
end = end_marker.encode()
inside = False
starts = 0
ends = 0
kept = []

for line in lines:
    content = line.rstrip(b"\r\n")
    if content == start:
        starts += 1
        if inside or starts > 1:
            raise SystemExit("duplicate or nested start marker in ~/.bashrc")
        inside = True
        continue
    if content == end:
        ends += 1
        if not inside or ends > 1:
            raise SystemExit("unmatched or duplicate end marker in ~/.bashrc")
        inside = False
        continue
    if not inside:
        kept.append(line)

if inside or starts != ends:
    raise SystemExit("incomplete Survellence_Sim marker block in ~/.bashrc")

result = b"".join(kept)
if action == "install":
    newline = b"\r\n" if b"\r\n" in raw else b"\n"
    if result and not result.endswith((b"\n", b"\r")):
        result += newline
    block = newline.join(
        (
            start,
            b"source ~/.survellence_sim.bash",
            end,
            b"",
        )
    )
    result += block

Path(output_path).write_bytes(result)
PY
    then
        rm -f "$temporary"
        fail "could not safely update the Survellence_Sim block in ~/.bashrc"
    fi
    mv "$temporary" "$bashrc"
}

if [[ "$mode" == "remove" ]]; then
    rewrite_bashrc remove
    rm -f "$registration_file"
    printf 'Removed the Survellence_Sim registration from ~/.bashrc and deleted ~/.survellence_sim.bash.\n'
    exit 0
fi

ros_setup="/opt/ros/jazzy/setup.bash"
[[ -f "$ros_setup" ]] || fail "ROS 2 Jazzy setup not found at $ros_setup"
source "$ros_setup" || fail "could not source $ros_setup"

if [[ "$mode" != "no-build" ]]; then
    (cd "$workspace_root" && colcon build --symlink-install) || fail "colcon build failed"
fi

install_setup="$workspace_root/install/setup.bash"
[[ -f "$install_setup" ]] || fail "workspace setup not found at $install_setup; build the workspace first"
source "$install_setup" || fail "could not source $install_setup"
set -u

package_prefix="$(ros2 pkg prefix surveillance_sim 2>/dev/null)" \
    || fail "surveillance_sim is not registered in this workspace"
perception_prefix="$(ros2 pkg prefix vision_perception 2>/dev/null)" \
    || fail "vision_perception is not registered in this workspace"
executables="$(ros2 pkg executables vision_perception 2>/dev/null)" \
    || fail "could not list vision_perception executables"
grep -Eq '^vision_perception[[:space:]]+yolo_node$' <<<"$executables" \
    || fail "vision_perception does not expose the yolo_node executable"
ros2 launch surveillance_sim surveillance_world.launch.py --show-args >/dev/null 2>&1 \
    || fail "surveillance_world.launch.py could not be resolved"

printf '%s\n' \
    '# Registered by scripts/register_with_ros.sh' \
    'case $- in *u*) _survellence_sim_restore_nounset=1; set +u ;; *) _survellence_sim_restore_nounset=0 ;; esac' \
    'source /opt/ros/jazzy/setup.bash' > "$registration_file.tmp"
printf 'source %q\n' "$install_setup" >> "$registration_file.tmp"
printf '%s\n' \
    'if [[ $_survellence_sim_restore_nounset -eq 1 ]]; then set -u; fi' \
    'unset _survellence_sim_restore_nounset' >> "$registration_file.tmp"
chmod 600 "$registration_file.tmp"
mv "$registration_file.tmp" "$registration_file"

if [[ ! -f "$bashrc" ]]; then
    : > "$bashrc"
fi
if [[ ! -e "$backup_file" ]]; then
    cp -p "$bashrc" "$backup_file"
fi
rewrite_bashrc install

if ! verification="$(
    bash -c '
        set -eo pipefail
        source ~/.survellence_sim.bash
        set -u
        ros2 pkg prefix surveillance_sim
        ros2 pkg prefix vision_perception
        executables="$(ros2 pkg executables vision_perception)"
        printf "%s\n" "$executables"
        grep -Eq "^vision_perception[[:space:]]+yolo_node$" <<<"$executables" \
            || { echo "yolo_node is missing from vision_perception" >&2; exit 1; }
        ros2 launch surveillance_sim surveillance_world.launch.py --show-args
    ' 2>&1
)"; then
    printf '%s\n' "$verification" >&2
    fail "fresh-shell ROS 2 verification failed"
fi

printf '%s\n' "$verification"
printf 'Registration complete for workspace: %s\n' "$workspace_root"
printf 'Open a new terminal or run: source ~/.bashrc\n'
