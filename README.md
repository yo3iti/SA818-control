# Programming SA818 module

**Before programming the SA818 module, make sure you consult the band plan for your country and transmit on a frequency you are allowed to use.**

## Purpose
This is a small application written in C for controlling SA818 radio module inside raspberry pi.
This works by sending various serial commands and configuring transmit and receive frequency, CTCSS tones, volume and some other parameters.

This is useful for ham radio and other electronics enthusiasts that want to have a small C application to setup their SA818 transceiver.
See here for more information: www.yo3iti.ro and www.alauda.ro

For stable version get the main.

## Build and installation

Please clone repo and use command `make` for build. To use the romanian language variation, please use `make LANG=ro`.

The target is names `sa818ctl`. You can modify the build by altering `makefile`. Use `make install` to install application in `/usr/bin`. 

## Usage

For help:
```
sa818ctl -h
```
or
```
sa818ctl --help
```
Some syntax examples are displayed in the help too

Basic syntax

```
sa818ctl -f 145.000 -t 103.5 -r 103.5
```
or
```
sa818ctl -freq 145.000 --txtone 103.5 --rxtone 103.5
```
for tail open/close
```
sa818ctl -a 1/0
```



