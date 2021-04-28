#
# Copyright (c) 2021, Ari Suutari <ari@stonepile.fi>.
# All rights reserved. 
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. The name of the author may not be used to endorse or promote
#     products derived from this software without specific prior written
#     permission. 
# 
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
# INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.10)

set(PORT msp430)
set(NANO 1)
#
# This is for TI ez430-rf2500 kit
#
#set(MCU msp430f2274)
#
# This is for Olimex MSP430-CCRF card
#
set(MCU cc430f5137)

if(MCU STREQUAL "msp430f2274")
set(TEXAS_CONF ${CMAKE_CURRENT_SOURCE_DIR}/../sensor-ap/texas)
set(RADIO MRFI_CC2500 NUM_CONNECTIONS=1 SIZE_INFRAME_Q=1)
endif()

if(MCU STREQUAL "cc430f5137")
set(TEXAS_CONF ${CMAKE_CURRENT_SOURCE_DIR}/../sensor-ap/texas_ccrf)
set(RADIO MRFI_CC430 NUM_CONNECTIONS=3 SIZE_INFRAME_Q=2)
set(ONEWIRE_DEFINES ONEWIRE SMALL_MEMORY_TARGET)
endif()

include(../picoos/cmake/ToolchainInit.cmake)

project(sensor-node)

set(DIR_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/config)

set(SRC  sensor_main.c)

add_peer_directory(${PICOOS_DIR})
add_peer_directory(../picoos-micro)
add_peer_directory(../picoos-ow)
add_peer_directory(../simpliciti)

target_compile_definitions(simpliciti PUBLIC 
	SIZE_OUTFRAME_Q=1
	THIS_DEVICE_ADDRESS={{0x79,0x56,0x34,0x12}}
	END_DEVICE
	MAX_HOPS=2
	MAX_HOPS_FROM_AP=1
	MAX_NWK_PAYLOAD=9
	MAX_APP_PAYLOAD=18
	DEFAULT_LINK_TOKEN=0x01020304
	DEFAULT_JOIN_TOKEN=0x05060708
	xEXTENDED_API APP_AUTO_ACK
	ISM_EU LINK_SPEED=1
	${RADIO}) 

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} simpliciti picoos-ow picoos-micro picoos)
target_compile_definitions(${PROJECT_NAME} PRIVATE ${ONEWIRE_DEFINES})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../sensor-ap)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                   COMMAND msp430-elf-size ${PROJECT_NAME}.elf)
