#!/bin/bash
# Install systemd service for auto-starting the room scanner

set -e

echo "============================================"
echo "Installing Room Scanner Systemd Service"
echo "============================================"

# Get directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SERVICE_TEMPLATE="$PROJECT_DIR/config/room-scanner.service"
SERVICE_FILE="/etc/systemd/system/room-scanner.service"

# Check if service template exists
if [ ! -f "$SERVICE_TEMPLATE" ]; then
    echo "Error: Service template not found at $SERVICE_TEMPLATE"
    exit 1
fi

# Replace placeholders in service file
echo "Creating service file..."
sed -e "s|%USER%|$USER|g" \
    -e "s|%PROJECT_DIR%|$PROJECT_DIR|g" \
    "$SERVICE_TEMPLATE" | sudo tee "$SERVICE_FILE" > /dev/null

# Set permissions
sudo chmod 644 "$SERVICE_FILE"

# Reload systemd
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo ""
echo "============================================"
echo "Service installed successfully!"
echo "============================================"
echo ""
echo "Service commands:"
echo "  Enable auto-start:  sudo systemctl enable room-scanner"
echo "  Start now:          sudo systemctl start room-scanner"
echo "  Stop:               sudo systemctl stop room-scanner"
echo "  Check status:       sudo systemctl status room-scanner"
echo "  View logs:          journalctl -u room-scanner -f"
echo ""
echo "To enable and start the service:"
echo "  sudo systemctl enable --now room-scanner"
echo ""
