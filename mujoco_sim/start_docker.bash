# starts docker container named mujoco with a s

# docker run -it --rm -v ./src:/mujoco/src mujoco
docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./src:/mujoco/src \
    mujoco