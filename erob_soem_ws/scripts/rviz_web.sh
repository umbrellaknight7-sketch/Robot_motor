#!/usr/bin/env bash
set -eo pipefail
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
viewer_logs="$workspace/log/dev/rviz_viewer"
case "${1:-status}" in
  start)
    pose="${2:-upright}"
    if [[ "$pose" != upright && "$pose" != zero ]]; then
      echo 'Initial pose must be upright or zero.' >&2; exit 2
    fi
    if tmux -L wheel-rviz has-session -t kinematics-rviz 2>/dev/null; then
      echo 'Viewer already running; stop it before selecting another initial pose.'; exit 0
    fi
    mkdir -p "$viewer_logs"
    printf -v command 'bash %q %q > %q 2>&1' "$workspace/scripts/rviz_viewer/run_viewer.sh" "$pose" "$viewer_logs/supervisor.log"
    tmux -L wheel-rviz new-session -d -s kinematics-rviz "$command"
    echo "Packaged CAD viewer starting with initial_pose=$pose."
    ;;
  stop)
    if tmux -L wheel-rviz has-session -t kinematics-rviz 2>/dev/null; then
      tmux -L wheel-rviz send-keys -t kinematics-rviz C-c
      echo 'Stop requested.'
    else echo 'Viewer is not running.'; fi
    ;;
  status)
    if tmux -L wheel-rviz has-session -t kinematics-rviz 2>/dev/null; then
      echo 'Session: kinematics-rviz; display :99; ROS domain 185; noVNC 127.0.0.1:6080'
      curl --fail --silent --output /dev/null http://127.0.0.1:6080/vnc.html && echo 'noVNC HTTP endpoint ready.'
    else echo 'Viewer is not running; check log/dev/rviz_viewer/supervisor.log.'; exit 1; fi
    ;;
  *) echo 'Usage: rviz_web.sh start [upright|zero] | stop | status' >&2; exit 2 ;;
esac
