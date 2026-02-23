#!/usr/bin/env bash
# Set script directory reliably and run the UI from src/main/scripts
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/src/main/scripts" || {
	echo "Cannot cd to $SCRIPT_DIR/src/main/scripts" >&2
	exit 1
}
python3 driving_mode_ui.py