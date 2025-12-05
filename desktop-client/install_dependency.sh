#!/bin/bash

# Update
sudo apt update

# Tools
sudo apt install -y build-essential cmake git

# Qt6
sudo apt install -y qt6-base-dev qt6-tools-dev qt6-tools-dev-tools

echo "All dependencies installed."
