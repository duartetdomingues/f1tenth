#!/bin/bash

# Directory containing service files in the repository
SERVICE_DIR="."  # Adjust this to the relative or absolute path of your folder

# Ensure the directory exists
if [ ! -d "$SERVICE_DIR" ]; then
  echo "Error: Directory $SERVICE_DIR does not exist."
  exit 1
fi
# Initialize an array to hold installed service names
services=()

# Loop through all .service files in the directory
for service_file in "$SERVICE_DIR"/*.service; do
  if [ -f "$service_file" ]; then
    # Copy the service file to systemd directory
    echo "Installing $(basename "$service_file")..."
    if systemctl list-unit-files | grep -q "^$(basename "$service_file")"; then
      echo "$(basename "$service_file") already exists in systemd. Skipping..."
      continue
    fi
    sudo cp "$service_file" /etc/systemd/system/
    service_name=$(basename "$service_file")
    services+=("$service_name")
  else
    echo "No .service files found in $SERVICE_DIR."
    exit 1
  fi
done

# Reload systemd to register the new services
echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

# Enable and start each service
for service_name in "${services[@]}"; do
  echo "Enabling and starting $service_name..."
  sudo systemctl enable "$service_name"
  sudo systemctl start "$service_name"
done

echo "All services installed, enabled, and started successfully."
