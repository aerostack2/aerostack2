all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build latest humble image"
	@echo "   2. make pull             - pull all latest humble image"
	@echo "   3. make clean            - remove all images"
	@echo ""

build:
	@docker build --no-cache --build-arg ROS_DISTRO=humble --build-arg IGNITION_VERSION=fortress --file=humble/Dockerfile --tag=as2:humble .

build-rolling:
	@docker build --no-cache --build-arg ROS_DISTRO=humble --build-arg IGNITION_VERSION=fortress --build-arg AEROSTACK2_BRANCH=main --file=rolling/Dockerfile --tag=as2:rolling .

build-galactic:
	@docker build --no-cache --build-arg ROS_DISTRO=galactic --build-arg IGNITION_VERSION="" --build-arg AEROSTACK2_BRANCH=main --file=rolling/Dockerfile --tag=as2:galactic .

pull:
	@docker pull as2:humble

clean:
	@docker rmi -f as2:humble
