#!/usr/bin/env bash
set -eo pipefail
viewer_tools=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
viewer_root=$(cd "$viewer_tools/../../.." && pwd)
viewer_logs="$viewer_root/erob_soem_ws/log/dev/rviz_viewer"
case "${1:-status}" in
  start)
    if tmux -L wheel-rviz has-session -t kinematics-rviz 2>/dev/null; then
      echo 'Viewer session already running.'; exit 0
    fi
    mkdir -p "$viewer_logs"
    printf -v command 'bash %q > %q 2>&1' "$viewer_tools/run_viewer.sh" "$viewer_logs/supervisor.log"
    tmux -L wheel-rviz new-session -d -s kinematics-rviz "$command"
    echo 'Viewer starting. Use status to check it; logs are in erob_soem_ws/log/dev/rviz_viewer/.'
    ;;
  stop)
    if tmux -L wheel-rviz has-session -t kinematics-rviz 2>/dev/null; then
      tmux -L wheel-rviz send-keys -t kinematics-rviz C-c
      echo 'Stop requested.'
    else echo 'Viewer is not running.'; fi
    ;;
  status)
    if tmux -L wheel-rviz has-session -t kinematics-rviz 2>/dev/null; then
      echo 'Session running: kinematics-rviz; display :99; VNC 127.0.0.1:5901; browser 127.0.0.1:6080'
      curl --fail --silent --output /dev/null http://127.0.0.1:6080/vnc.html && echo 'noVNC HTTP endpoint ready.'
    else echo 'Viewer is not running. Check supervisor.log.'; exit 1; fi
    ;;
  *) echo 'Usage: rviz_web.sh start|stop|status' >&2; exit 2 ;;
esac
