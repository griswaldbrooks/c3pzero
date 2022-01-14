docker run -it \
  --hostname="cp30-dev" \
  -v "$HOME"/.ssh:"$HOME"/.ssh \
  -v "$PWD":"$HOME"/ws/src \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=89 \
  --privileged \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  --net=host \
  c3pzero-dev \
  bash
