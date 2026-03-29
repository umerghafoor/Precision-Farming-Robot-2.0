#!/bin/bash

# Raspberry Pi ROS2 Humble Network Setup
# This script configures ROS2 for a shared DDS network with Laptop Docker

set -e

# -----------------------------
# User-configurable variables
# -----------------------------
ROS_DOMAIN_ID=0                     # Must match Laptop Docker
PI_IP="${PI_IP:-192.168.0.112}"     # Pi LAN IP
LAPTOP_IP="${LAPTOP_IP:-192.168.0.113}"  # Laptop Docker LAN IP
CYCLONEDDS_CONFIG="$HOME/.ros/dds_config.xml"

# -----------------------------
# Create .ros folder if missing
# -----------------------------
mkdir -p "$(dirname "$CYCLONEDDS_CONFIG")"

# -----------------------------
# Generate CycloneDDS config
# -----------------------------
cat > "$CYCLONEDDS_CONFIG" <<EOF
<?xml version="1.0"?>
<dds>
  <Domain id="${ROS_DOMAIN_ID}">
    <General>
      <NetworkInterfaceAddress>${PI_IP}</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="${LAPTOP_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</dds>
EOF

# -----------------------------
# Export environment variables
# -----------------------------
echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> ~/.bashrc
echo "export CYCLONEDDS_URI=${CYCLONEDDS_CONFIG}" >> ~/.bashrc
source ~/.bashrc

# -----------------------------
# Done
# -----------------------------
echo "--------------------------------------------"
echo "✅ Raspberry Pi ROS2 network setup complete"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "CycloneDDS config: ${CYCLONEDDS_CONFIG}"
echo "Pi IP: ${PI_IP}"
echo "Laptop Docker IP (Peer): ${LAPTOP_IP}"
echo "--------------------------------------------"

echo "You can now run ROS2 nodes on this Pi and they will communicate with Docker container on the Laptop."