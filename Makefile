ROS_IP := 127.0.0.1

.PHONY: .compile

install-from-hub:
	docker pull personalroboticsimperial/uni_lace:latest
	@$(MAKE) -s .compile

install-from-source:
	docker build -t personalroboticsimperial/uni_lace:latest .
	@$(MAKE) -s .compile

.compile:
	git -C ${PWD}/catkin_ws/src clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
	docker container stop uni_lace || true && docker container rm uni_lace || true
	docker run \
		-it \
		-e ROS_IP="${ROS_IP}" \
		-e ROS_MASTER_URI="http://${ROS_IP}:11311" \
		-e DISPLAY \
    	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /dev:/dev \
		-v ${PWD}/catkin_ws:/catkin_ws:rw \
		-v ${PWD}:/UniLace:rw \
		--detach \
		--privileged \
		--runtime nvidia \
		--network host \
  		--gpus all \
		--name uni_lace \
		personalroboticsimperial/uni_lace:latest
	docker exec uni_lace bash -c "source /opt/ros/noetic/setup.bash && catkin build"
	docker container stop uni_lace

demo:
	xhost +si:localuser:root >> /dev/null
	docker start uni_lace
	sleep 1
	docker exec -it uni_lace bash -c "source devel/setup.bash && cd /UniLace/user_gym_policy && python uni_lace_demo.py"
	docker container stop uni_lace 

start-live:
	xhost +si:localuser:root >> /dev/null
	docker start uni_lace
	docker exec -it uni_lace bash -c "source devel/setup.bash && roslaunch uni_lace uni_lace_live.launch"
	docker container stop uni_lace

start-gym:
	xhost +si:localuser:root >> /dev/null
	docker start uni_lace
	sleep 1
	docker exec -it uni_lace bash -c "source devel/setup.bash && cd /UniLace/user_gym_policy && python uni_lace_gym_simple_policy.py"
	docker container stop uni_lace

install-baseline:
	git -C ${PWD}/catkin_ws/src clone git@github.com:ImperialCollegeLondon/robot_sl.git
	git -C ${PWD}/catkin_ws/src clone git@github.com:ImperialCollegeLondon/yumi-moveit.git
	docker start uni_lace
	sleep 1
	docker exec uni_lace bash -c "source /opt/ros/noetic/setup.bash && catkin build"
	docker container stop uni_lace 

run-baseline:
	xhost +si:localuser:root >> /dev/null
	docker start uni_lace
	docker exec -it uni_lace bash -c "source devel/setup.bash && roslaunch sl_ctrl robot_sl.launch sim:=true"

debug:
	xhost +si:localuser:root >> /dev/null
	docker start uni_lace
	docker exec -it uni_lace bash -c "source devel/setup.bash && bash"

recompile:
	docker start uni_lace
	docker exec -it uni_lace bash -c "source /opt/ros/noetic/setup.bash && catkin build"
	docker container stop uni_lace

stop:
	docker container stop uni_lace 

push:
	docker push personalroboticsimperial/uni_lace