export UUID=$(id -u)
export UGID=$(id -g)

docker build \
  -f Dockerfile \
  -t c3pzero-dev:latest \
  --build-arg USERNAME=$USER \
  --build-arg USER_UID=$UUID \
  --build-arg USER_GID=$UGID \
  --no-cache \
  .
