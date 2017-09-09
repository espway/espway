# ESPway
A segway-like balancing two-wheel robot built on ESP8266. It is controlled over WiFi via a HTML/JS GUI and a WebSocket. This is a work in progress. The firmware is meant to run on a WEMOS D1 mini or a similar board.

**NOTE for existing users**: the [Arduino version](https://github.com/flannelhead/espway/tree/arduino) of this code is deprecated and will *not* receive the newest improvements from the main branch. However, the Arduino branch will remain in this repo. If you'd like to have something improved or fixed there, please open an issue or preferably a pull request.

## Getting started
The firmware is built on the [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos) project which is included as a submodule in this repo.

Building ESPway is easiest on a Unix-based operated system. To build the firmware binary, you'll need the following tools available in the path:

* `git`
* `esp-open-sdk`, see the [instructions](https://github.com/SuperHouse/esp-open-rtos/#quick-start) in the esp-open-rtos repo for the specific make command
* `perl`, `nodejs` and `npm` if you wish to hack on the frontend HTML/JS code

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

Compared to Arduino, some might find it a bit cumbersome to get started due to the need to build an entire toolchain and operate in the command line. For the toolchain, I intend to provide a minimal Docker image if that doesn't turn out too cumbersome. Moreover, I think it would not be a bad idea to ship prebuilt binaries of the firmware and offer some kind of a simple flashing tool, essentially wrapping `esptool`.

## Supported browsers
Please use the latest Firefox or Chrome if possible. The HTML/JS UI uses some
recent JavaScript features which might not be supported by older browsers. However if you encounter any issues on e.g. Safari, don't hesitate to file them in the issue tracker.

## Schematic & BOM

A PCB design and a 3D printable body are planned, but nothing has been done yet. The intent is to have a leaner design, replacing the D1 Mini board with a smaller ESP-12S module and leaving out the USB port. Also MPU6050 will likely be replaced by LSM6DS3 due to better availability and cheaper price as discrete chips.

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

See the `schematic` folder for the schematic. It is drawn with [KiCad](http://kicad-pcb.org/) and there's a [rendered PDF](https://github.com/flannelhead/espway/raw/master/schematic/espway.pdf) in the repo.

## Developing the frontend
The HTML/JS frontend uses [Webpack](https://webpack.github.io/) as the build system. You will need [NodeJS](https://nodejs.org/en/) and NPM (the package manager) to build the frontend pages. It does jobs like bundling the JavaScript modules together, minifying and transpiling the [ES2015](https://babeljs.io/learn-es2015/) code for older browsers, compiling [Riot tags](http://riotjs.com/), minifying/autoprefixing CSS etc.

After you've cloned this repo, run `npm install` in the root folder to install the package dependencies. There are two commands specified in `package.json` which run Webpack:

* `npm run serve` start a web server which serves the static HTML files in the `frontend/output` directory. It also watches for changes in the source files in `frontend/src` and automatically rebuilds and reloads them. Use this while hacking on the frontend. *TODO*: Currently this doesn't see changes to HTML files, though.
* `npm run build` produces a production build of the frontend JS/CSS bundles. Use this before uploading your work to the ESP8266.

## License
The project is licensed under GPLv3.

The project uses `esp-open-rtos` which is licensed under the BSD 3-clause license. It also has some components which have different licenses. Read more about them in the [`esp-open-rtos` README](https://github.com/SuperHouse/esp-open-rtos/blob/master/README.md).
