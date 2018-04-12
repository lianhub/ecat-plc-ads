#!/bin/bash

cd /home/pi/ecat-plc-ads/plc
echo Generating object files...
g++ -I ./lib -c Config0.c
g++ -I ./lib -c Res0.c
echo Generating glueVars.cpp
./glue_generator

cd ..
echo Compiling main program
g++ *.cpp ./ads/*.cpp ./plc/*.cpp ./plc/Res0.o ./plc/Config0.o -o openplc -I ./plc -lethercat -lrt -pthread -fpermissive

cd /home/pi/OpenPLC_v2
