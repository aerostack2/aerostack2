all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all images"
	@echo "   2. make pull             - pull all images"
	@echo "   3. make clean            - remove all images"
	@echo ""

build:
	@docker build --tag=as2:humble --file=docker/Dockerfile-humble .

pull:
	@docker pull as2:humble

clean:
	@docker rmi -f as2:humble
