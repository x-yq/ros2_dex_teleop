# on personal mac
# docker image: teleop_data_dev
# container name: teleop_data_dev_container
docker run -it \
  --name teleop_data_dev_container \
  --network=host \
  --ipc=host \
  --privileged \
  --security-opt seccomp=unconfined \
  --cap-add=ALL \
  --user=root \
  --workdir=/root \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$HOME/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v $(pwd):/src \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /run/udev:/run/udev \
  -v /lib/modules:/lib/modules \
  -v /sys:/sys \
  --device=/dev \
  --device-cgroup-rule='c *:* rmw' \
  --pid=host \
  teleop_data_dev \
  bash