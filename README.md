# ESPway
A segway-like balancing two-wheel robot built on ESP8266. It is controlled over WiFi via a HTML/JS GUI and a WebSocket. This is a work in progress. The firmware is meant to run on a WEMOS D1 mini or a similar board with 4 megabytes of flash.

## Building
This project is meant to be built against the ESP8266 NONOS SDK V2.0.0 20160810. You can use the [`esp-open-sdk` Makefile](https://github.com/pfalcon/esp-open-sdk) to setup the SDK and compiler toolchain. A Unix-based platform is recommended for building.

If this is your first time flashing this code to the WEMOS D1 mini, run
```
make blankflash
```
to upload initial parameters required by the firmware.

Then you can build and flash the firmware by running
```
make clean && make && make flash
```
with the WEMOS D1 mini connected. This will upload both the firmware and the
filesystem containing the HTML UI.

## Developing the frontend
The HTML/JS frontend uses [Webpack](https://webpack.github.io/) as the build system. You will need [NodeJS](https://nodejs.org/en/) and NPM (the package manager) to build the frontend pages. It does jobs like bundling the JavaScript modules together, minifying and transpiling the [ES2015](https://babeljs.io/learn-es2015/) code for older browsers, compiling [Riot tags](http://riotjs.com/), minifying/autoprefixing CSS etc.

After you've cloned this repo, run `npm install` in the root folder to install the package dependencies. There are two commands specified in `package.json` which run Webpack:

* `npm run serve` start a web server which serves the static HTML files in the `html` directory. It also watches for changes in the source files in `html-src` and automatically rebuilds and reloads them. Use this while hacking on the frontend.
* `npm run build` produces a production build of the frontend JS/CSS bundles. Use this before uploading your work to the ESP8266.

If the frontend on the ESP is not updated for some reason when flashing, please run `make clean` to ensure the flash filesystem gets rebuilt.

## Dependencies
The project utilizes some third-party libraries, in their respective folders:

* [libesphttpd](https://github.com/Spritetm/libesphttpd) by Jeroen Domburg (Spritetm) under "THE BEER-WARE LICENSE"
* [ESP8266_new_pwm](https://github.com/StefanBruens/ESP8266_new_pwm) by StefanBruens under GPLv2-or-later
* [esp8266ws2812i2s](https://github.com/cnlohr/esp8266ws2812i2s) by Charles Lohr, under the Espressif MIT License, permitting use on ESP8266 systems only
* [brzo_i2c](https://github.com/pasko-zh/brzo_i2c), a fast assembly I2C master implementation by Pascal Kurtansky, under GLPv3

Kudos to all the above developers. Without these libraries this project wouldn't exist in such quality. The libraries are intentionally included in this repo because I had to make some minor adjustments to be able to build them under the NONOS SDK and strict `-Wall -Werror` compile flags.

[Riot.js](http://riotjs.com/) is used on the HTML+JS frontend for facilitating some UI components. Other compile-time JavaScript dependencies are listed in `package.json`.

## Schematic & BOM

See the `schematic` folder for the schematic. It is drawn with [KiCad](http://kicad-pcb.org/) and there's a [rendered PDF](https://github.com/flannelhead/espway/raw/master/schematic/espway.pdf) in the repo.

Tentative BOM (not including PCB, connectors, wire etc. materials):

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

## Supported browsers

Please use the latest Firefox or Chrome if possible. The HTML/JS UI uses some
recent JavaScript features which might not be supported by older browsers.

## License
The project is licensed under GPLv3.

