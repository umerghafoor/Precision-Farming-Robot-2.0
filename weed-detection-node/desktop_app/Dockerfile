FROM ros:jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-pyqt6 \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt

# Create app directory
WORKDIR /root/desktop_app

# Copy application files
COPY . /root/desktop_app/

# Set environment
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

# Run the application
CMD ["python3", "main.py"]

