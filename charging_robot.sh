#!/bin/bash
PROJECT_DIR="/home/argoon/Project/Charging_Robot_Operation"
VENV_DIR="$PROJECT_DIR/charging_robot"

# 가상환경 활성화 (이미 활성화되어 있으면 생략)
if [ -z "$VIRTUAL_ENV" ]; then
    source "$VENV_DIR/bin/activate"
fi

cd "$PROJECT_DIR"
python scripts/main.py "$@"
