#!/bin/bash

xhost +local:docker
 docker start hokuyo_navigation2_dev 
 docker exec -it hokuyo_navigation2_dev /bin/bash
