TOOLCHAIN_BUILD_DIR = stm32f3-discovery-template

SRC_DIR = $(shell pwd)/src
INC_DIR = $(shell pwd)/inc

.PHONY : clean build flash

clean:
	$(MAKE) -C $(TOOLCHAIN_BUILD_DIR) reallyclean

build:
	$(MAKE) -C $(TOOLCHAIN_BUILD_DIR) SRC_DIR=$(SRC_DIR) INC_DIR=$(INC_DIR) all

flash: build
	$(MAKE) -C $(TOOLCHAIN_BUILD_DIR) program
