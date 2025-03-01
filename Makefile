DOCKER_IMAGE_NAME = machina_metrology_task
CONTAINER_NAME = ros2_jazzy_dev

.PHONY: build
build:
	docker build -t $(DOCKER_IMAGE_NAME) ../src

.PHONY: run
run:
	xhost +local:root  # Allow X11 forwarding
	docker run -it --rm --name $(CONTAINER_NAME) \
		--net=host \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /dev/shm:/dev/shm \
		--device=/dev/dri:/dev/dri \
		-v $(shell pwd)/../src:/ros2_ws/src \
		-v $(shell pwd)/../src/machina_metrology_task/data/calibration:/ros2_ws/src/machina_metrology_task/data/calibration \
		-v $(shell pwd)/../src/machina_metrology_task/data/pcd:/ros2_ws/src/machina_metrology_task/data/pcd \
		-v $(shell pwd)/../src/machina_metrology_task/output:/ros2_ws/src/machina_metrology_task/output \
		-v $(shell pwd)/../src/machina_metrology_task/config:/ros2_ws/src/machina_metrology_task/config \
		$(DOCKER_IMAGE_NAME)

.PHONY: bash
bash:
	docker exec -it $(CONTAINER_NAME) bash

.PHONY: clean
clean:
	docker rmi -f $(DOCKER_IMAGE_NAME)
