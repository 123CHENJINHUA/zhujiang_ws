docker run -it --rm \
  --privileged=true \
  --name zhujiang \
  --network=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e LANG=C.UTF-8 \
  -e LC_ALL=C.UTF-8 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -v /dev/snd:/dev/snd \
  -v /dev/dri:/dev/dri \
  -v /home/$USER:/home/$USER \
  --group-add audio \
  --group-add video \
  zhujiang:v1