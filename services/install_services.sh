#!/bin/bash

# Directory containing service files in the repository
SERVICE_DIR="."  # Adjust this to the relative or absolute path of your folder

# Ensure the directory exists
if [ ! -d "$SERVICE_DIR" ]; then
  echo "Error: Directory $SERVICE_DIR does not exist."
  exit 1
fi

# Loop through all .service files in the directory
for service_file in "$SERVICE_DIR"/*.service; do
  if [ -f "$service_file" ]; then
    # Copy the service file to systemd directory
    echo "Installing $(basename "$service_file")..."
    sudo cp "$service_file" /etc/systemd/system/
  else
    echo "No .service files found in $SERVICE_DIR."
    exit 1
  fi
done

# Reload systemd to register the new services
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable and start each service
for service_file in "$SERVICE_DIR"/*.service; do
  service_name=$(basename "$service_file")
  echo "Enabling and starting $service_name..."
  sudo systemctl enable "$service_name"
  sudo systemctl start "$service_name"
done

echo "All services installed, enabled, and started successfully."
