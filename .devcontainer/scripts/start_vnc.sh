#!/bin/bash

echo ">>> CONFIGURING AND STARTING VNC <<<"

# 1. Clean up old X11 locks
sudo rm -rf /tmp/.X11-unix/X1
sudo rm -rf /tmp/.X1-lock

# 2. Set up XDG_RUNTIME_DIR for Qt/Gazebo
export XDG_RUNTIME_DIR=/tmp/runtime-developer
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# 3. Create the VNC configuration directory
mkdir -p ~/.vnc

# 4. Create the startup file for the XFCE desktop
cat <<EOF > ~/.vnc/xstartup
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
unset WAYLAND_DISPLAY
unset I3SOCK
export DISPLAY=:1
export XKL_XMODMAP_DISABLE=1
exec dbus-launch --exit-with-session startxfce4
EOF

# 5. Make the startup script executable
chmod +x ~/.vnc/xstartup

# 6. Start the VNC server
vncserver :1 -geometry 1920x1080 -depth 24 -SecurityTypes None

# 7. Start the noVNC web bridge in the background (with nohup so it survives)
nohup websockify --web /usr/share/novnc/ 6080 localhost:5901 > /tmp/websockify.log 2>&1 &

# 8. Auto-export variables for future terminal sessions
if ! grep -q "DISPLAY=:1" ~/.bashrc; then
    echo "export DISPLAY=:1" >> ~/.bashrc
fi
if ! grep -q "XDG_RUNTIME_DIR=/tmp/runtime-developer" ~/.bashrc; then
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-developer" >> ~/.bashrc
fi

echo ">>> VNC SERVER RUNNING <<<"
echo "Access the desktop in your browser at: http://localhost:6080/vnc.html"