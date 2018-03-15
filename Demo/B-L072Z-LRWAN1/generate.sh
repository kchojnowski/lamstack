#!/bin/bash

autoreconf --install
./configure --host=arm-none-eabi CXXFLAGS="-specs=nosys.specs"

