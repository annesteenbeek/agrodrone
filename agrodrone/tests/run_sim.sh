#!/bin/bash

source ~/.rosrc
cd ~/src/ardupilot/ArduCopter
sim_vehicle.py -m --dialect=ardupilotmega -m --load-module=agrodrone --console --map
