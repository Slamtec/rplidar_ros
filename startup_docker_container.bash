#!/bin/bash

# clear

docker build -t slamtec_lidar .

docker stop slamtec_lidar

docker rm slamtec_lidar

docker run -it \
--env-file runtime.env \
--restart unless-stopped \
--network=host \
--name=slamtec_lidar  \
slamtec_lidar:latest  