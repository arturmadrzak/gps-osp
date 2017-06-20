## gps-osp

Library is an implemetation of SirfStar IV protocol, known as One Socket Protocol(OSP) or earlier SirfBinary. For testing purposes I used module with software version: "6GSD4e_4.1.2-P1 R+"

## dependencies

OSP receiver depends on https://gitlab.com/madrypl/driver. In makefle is hardcoded patch to compiled library relative to this directory (../).

## build

build ```driver```
make 

## example

Simple program demonstrating how use a library. By default all(most at least) SirfStar modules use NMEA@4800bps. To switch module into OSP mode issue:
```
./example -t /dev/ttyUSB0 -o -n
```

