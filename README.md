# Neato Botvac
This repository provides ROS2 packages intended for in and out communication with a Neato Botvac.

Only tested on a D-series (either 80 or 85)

## Developing

Note to self: Command currently using. TODO push docker image for others to use

```
docker run -it --rm \
    --name neato \
    --device=/dev/ttyACM0 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v $(pwd):/ws \
    -w /ws \
    homeydev
```
