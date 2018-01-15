# ESPway
A segway-like balancing two-wheel robot built on ESP8266. It is controlled over WiFi via a HTML/JS GUI and a WebSocket. This is a work in progress. The firmware is meant to run on a WEMOS D1 mini or a similar board.

**NOTE for existing users**: the [Arduino version](https://github.com/flannelhead/espway/tree/arduino) of this code is deprecated and will *not* receive the newest improvements from the main branch. However, the Arduino branch will remain in this repo. If you'd like to have something improved or fixed there, please open an issue or preferably a pull request.

## Getting started
The firmware is built on the [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos) project which is included as a submodule in this repo.

For building the firmware, a Linux host is recommended. For other platforms, there is a [Docker image](https://hub.docker.com/r/flannelhead/espway-toolchain/) one can use. The `docker-run` script in this repository contains the command that's required to download the image and run commands in the container. It should work fine on macOS hosts. For Windows hosts, I still recommend installing Linux in a virtual machine and installing the tools there.

### Installing the tools and building

*NOTE*: if you intend to use Docker, you only have to install `esptool`, `git` and `make` on your host. Please see below for further Docker instructions.

Install these packages with your distribution's package manager:
* `git`
* `xtensa-lx106-elf` toolchain. You can use `esp-open-sdk` to build it, see the [instructions](https://github.com/SuperHouse/esp-open-rtos/#quick-start) in the esp-open-rtos repo
* `esptool` (`pip install -U esptool`). Please do not use pulled by `esp-open-sdk`, it is outdated.
* `tcc`, `nodejs` and `npm` are currently required due to the frontend code, although I'm investigating how to relax this dependency.

Clone this repo (recursive cloning to get also `esp-open-rtos` and its submodules):
```
git clone --recursive https://github.com/flannelhead/espway.git
```
Enter the directory and build the firmware:
```
make parallel
```
The `parallel` target does a clean build with 5 parallel worker threads to make it faster.

Plug your ESP8266 module in via USB and flash the firmware:
```
make flash
```
The default port is `/dev/ttyUSB0`. If you need to change this, use
```
make flash ESPPORT=/dev/ttyUSBx
```

### Using the Docker image

First, [install Docker](https://www.docker.com/community-edition).

Building the firmware using the supplied Docker image is easy. Instead of running `make parallel`, just run `./docker-run make parallel` in the root folder of the repo. The command supplied to the script will be run in the Docker container, and the image will be automatically downloaded.

Flashing the image differs, though, and for this you'll need `make` on your host. Instead of
```
make flash ESPPORT=/dev/ttyUSBx
```
(`/dev/ttyUSBx` being the ESP's serial port) run
```
make flash-only ESPPORT=/dev/ttyUSBx
```
in your host shell. The separate `flash-only` target is needed because the `flash` target would try to build the firmware. In the future, it is intended to provide a separate Python script for flashing, lifting the need for `make` on host.

## Supported browsers
Please use the latest Firefox or Chrome if possible. The HTML/JS UI uses some
recent JavaScript features which might not be supported by older browsers. However if you encounter any issues on e.g. Safari, don't hesitate to file them in the issue tracker.

## Schematic & BOM

A PCB design and a 3D printable body are planned. There is an initial design already in the `schematic` folder, but please don't use it quite yet - I'm still waiting for the (hopefully) final iteration of the board. There, MPU6050 will be replaced by LSM6DS3 and L293D for DRV8835 for better performance.

Meanwhile, you can still build an ESPway using breakout boards available from the usual sources. A rough bill of materials for this is listed below:

* (not including PCB, connectors, wire etc. materials)
* WEMOS D1 Mini board
* GY-521 (MPU6050 breakout board)
* L293D motor driver IC. **N.B.** this is what I used so far, but I'm intending to change to DRV8833 for a smaller voltage drop across the H-bridge
* 2x 6V 300rpm metal gear motor (search for "12ga 300rpm" or "n20 300rpm"), these should be $3-5 per piece
* 2x WS2812B neopixels for eyes and showing current state
* AMS1117 5V regulator
* 5x 100n ceramic capacitors
* 2x 1000u 10V electrolytic capacitor
* 470 ohm resistor
* 10 kohm resistor
* 680 kohm resistor

See the `schematic-old` folder for the schematic. It is drawn with [KiCad](http://kicad-pcb.org/) and there's a [rendered PDF](https://github.com/flannelhead/espway/raw/master/schematic/espway.pdf) in the repo.

There's a new schematic in the works in `schematic` folder. It uses components
from [kicad-ESP8266](https://github.com/jdunmire/kicad-ESP8266) by J. Dunmire,
licensed under CC-BY-SA 4.0. The schematic is also licensed under CC-BY-SA 4.0.

## Developing the frontend
The HTML/JS frontend uses [Webpack](https://webpack.github.io/) as the build system. You will need [NodeJS](https://nodejs.org/en/) and NPM (the package manager) to build the frontend pages. It does jobs like bundling the JavaScript modules together, minifying and transpiling the [ES2015](https://babeljs.io/learn-es2015/) code for older browsers, compiling [Riot tags](http://riotjs.com/), minifying/autoprefixing CSS etc.

After you've cloned this repo, run `npm install` in the root folder to install the package dependencies. There are two commands specified in `package.json` which run Webpack:

* `npm run serve` start a web server which serves the static HTML files in the `frontend/output` directory. It also watches for changes in the source files in `frontend/src` and automatically rebuilds and reloads them. Use this while hacking on the frontend. *TODO*: Currently this doesn't see changes to HTML files, though.
* `npm run build` produces a production build of the frontend JS/CSS bundles. Use this before uploading your work to the ESP8266.

## License and acknowledgements
The project is licensed under LGPLv3.

The project uses `esp-open-rtos` which is licensed under the BSD 3-clause license. It also has some components which have different licenses. Read more about them in the [`esp-open-rtos` README](https://github.com/SuperHouse/esp-open-rtos/blob/master/README.md).
