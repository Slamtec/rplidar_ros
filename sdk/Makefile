#/*
# *  RPLIDAR SDK
# *
# *  Copyright (c) 2009 - 2014 RoboPeak Team
# *  http://www.robopeak.com
# *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
# *  http://www.slamtec.com
# *
# */
#/*
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * 1. Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# * 2. Redistributions in binary form must reproduce the above copyright notice,
# *    this list of conditions and the following disclaimer in the documentation
# *    and/or other materials provided with the distribution.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
# */
#
HOME_TREE := ../

MODULE_NAME := $(notdir $(CURDIR))

include $(HOME_TREE)/mak_def.inc

CXXSRC += src/sl_lidar_driver.cpp \
          src/hal/thread.cpp\
          src/sl_crc.cpp\
	      src/sl_serial_channel.cpp\
	      src/sl_tcp_channel.cpp\
	      src/sl_udp_channel.cpp

C_INCLUDES += -I$(CURDIR)/include -I$(CURDIR)/src

ifeq ($(BUILD_TARGET_PLATFORM),Linux)
CXXSRC += src/arch/linux/net_serial.cpp \
          src/arch/linux/net_socket.cpp \
          src/arch/linux/timer.cpp 
endif


ifeq ($(BUILD_TARGET_PLATFORM),Darwin)
CXXSRC += src/arch/macOS/net_serial.cpp \
          src/arch/macOS/net_socket.cpp \
          src/arch/macOS/timer.cpp
endif

all: build_sdk

include $(HOME_TREE)/mak_common.inc

clean: clean_sdk
