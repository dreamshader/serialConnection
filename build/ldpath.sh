#!/bin/bash
#
export LD_LIBRARY_PATH=/home/dirk/Projects/Current/Linux/serialConnection/build:$LD_LIBRARY_PATH


sudo LD_LIBRARY_PATH=~/Projects/Current/Linux/serialConnection/build:$LD_LIBRARY_PATH ./serialTest -c /dev/ttyS0


