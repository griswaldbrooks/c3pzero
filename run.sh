docker run -it \
  --hostname="c3pzero-dev" \
  -v "$HOME"/.ssh:"$HOME"/.ssh \
  -v "$PWD":"$HOME"/ws/src \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
  -v "/etc/group:/etc/group:ro" \
  -v "/etc/passwd:/etc/passwd:ro" \
  -v "/etc/shadow:/etc/shadow:ro" \
  --group-add=dialout \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=89 \
  --privileged \
  --device=/dev/ttyUSB0 \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  --net=host \
  c3pzero-dev \
  bash
