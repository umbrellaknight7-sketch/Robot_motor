#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/environment.sh"
viewer_tools="$viewer_package/tools"
deps="$viewer_artifacts/deps"
run="$viewer_artifacts/run"
mkdir -p "$run"
chmod 700 "$run"
export DISPLAY=:99 XAUTHORITY="$run/Xauthority"
export XDG_RUNTIME_DIR="$run/runtime"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
if [[ -e /tmp/.X99-lock ]]; then echo 'Display :99 is occupied; refusing to replace another X server.'; exit 1; fi
python - <<'PY'
import socket
for port in (5901, 6080):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', port))
PY
umask 077
touch "$XAUTHORITY"
xauth -f "$XAUTHORITY" add "$DISPLAY" . "$(python -c 'import secrets; print(secrets.token_hex(16))')"
if [[ ! -s "$run/vnc.passwd" ]]; then
  python - "$deps/usr/bin/x11vnc" "$run" <<'PY'
from pathlib import Path
import secrets, string, subprocess, sys
path=Path(sys.argv[2])
password=''.join(secrets.choice(string.ascii_letters+string.digits) for _ in range(8))
(path/'vnc_password.txt').write_text(password+'\n')
subprocess.run([sys.argv[1],'-storepasswd',password,str(path/'vnc.passwd')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
PY
fi
children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${children[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
"$deps/usr/bin/Xvfb" "$DISPLAY" -screen 0 1600x1000x24 -nolisten tcp -noreset -auth "$XAUTHORITY" +extension GLX +render > "$viewer_logs/xvfb.log" 2>&1 &
children+=("$!")
for attempt in {1..100}; do
  if xdpyinfo >/dev/null 2>&1; then break; fi
  sleep .1
done
xdpyinfo >/dev/null
env LD_LIBRARY_PATH="$deps/usr/lib/x86_64-linux-gnu" XDG_DATA_DIRS="$deps/usr/share:/usr/share" \
  "$deps/usr/bin/openbox" --config-file "$viewer_tools/openbox.xml" > "$viewer_logs/window-manager.log" 2>&1 &
children+=("$!")
"$deps/usr/bin/x11vnc" -display "$DISPLAY" -auth "$XAUTHORITY" -localhost -listen 127.0.0.1 -no6 \
  -rfbport 5901 -rfbauth "$run/vnc.passwd" -forever -shared -noxdamage -repeat -norc \
  > "$viewer_logs/vnc.log" 2>&1 &
children+=("$!")
env PYTHONPATH="$deps/usr/lib/python3/dist-packages" python -m websockify \
  --web "$deps/usr/share/novnc" --heartbeat 30 127.0.0.1:6080 127.0.0.1:5901 \
  > "$viewer_logs/novnc.log" 2>&1 &
children+=("$!")
ros2 launch kinematics_only display.launch.py gui:=true preview_limits:=true > "$viewer_logs/ros-launch.log" 2>&1 &
children+=("$!")
# Give each GUI its own area. No panel, desktop session or terminal is launched.
for attempt in {1..120}; do
  windows=$("$deps/usr/bin/wmctrl" -l 2>/dev/null || true)
  if [[ "$windows" == *"RViz"* && "$windows" == *"Joint State Publisher"* ]]; then
    "$deps/usr/bin/wmctrl" -r 'RViz' -b remove,maximized_vert,maximized_horz
    "$deps/usr/bin/wmctrl" -r 'RViz' -e 0,0,0,1110,960
    "$deps/usr/bin/wmctrl" -r 'Joint State Publisher' -e 0,1120,0,470,960
    break
  fi
  sleep .5
done
# Any failed component ends the session and cleans up its siblings.
wait -n "${children[@]}"
