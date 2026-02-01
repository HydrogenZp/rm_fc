#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
SERVICE_NAME="slam_startup"

chmod +x ${SCRIPT_DIR}/slam_startup.sh

sudo cp ${SCRIPT_DIR}/${SERVICE_NAME}.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}.service

echo "Service installed. Commands:"
echo "  sudo systemctl start ${SERVICE_NAME}"
echo "  sudo systemctl stop ${SERVICE_NAME}"
echo "  sudo systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"
