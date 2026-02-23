<<<<<<< HEAD
#!/usr/bin/env bash
# Set script directory reliably and run the UI from src/main/scripts
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/src/main/scripts" || {
	echo "Cannot cd to $SCRIPT_DIR/src/main/scripts" >&2
	exit 1
}
python3 driving_mode_ui.py
=======
cd ~/aim_ws/src/main/scripts
python3 driving_mode_ui.py
>>>>>>> 8ec809943bb73b8cc7e6b9e8f6287a9ded555244
