# ESPway
A segway-like balancing two-wheel robot built on ESP8266. It is controlled over WiFi via a HTML/JS GUI and a WebSocket. This is a work in progress. The firmware is meant to run on a WEMOS D1 mini or a similar board.

## Building
The firmware is meant to be built with Arduino IDE using the ESP8266
environment. Things you need to install:

* [Arduino IDE](https://www.arduino.cc/en/Main/Software)
* [ESP8266 core for Arduino](https://github.com/esp8266/arduino)
  [2.4.0-rc1](https://github.com/esp8266/Arduino/releases/tag/2.4.0-rc1)
* [arduino-esp8266fs-plugin](https://github.com/esp8266/arduino-esp8266fs-plugin)
* [NeoPixelBus](https://github.com/Makuna/NeoPixelBus) library
* [ESPAsyncTCP](https://github.com/me-no-dev/ESPAsyncTCP) library
* [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) library

Since the latter two libraries are somewhat critical to make the robot tick and no stable releases exist, I'm inclined to add them as submodules locally in the repo.

Steps needed to build:

1. Open the file `espway-arduino.ino` in the Arduino IDE.
2. Choose your board from the list in *Tools -> Board*. If you have a choice between 1M
   and 3M SPIFFS size in *Tools -> Board Size*, I recommend the 1M to shorten the upload time.
3. *Sketch -> Upload* to upload the sketch.
4. *Tools -> ESP8266* Sketch Data Upload to upload the HTML UI.

## Supported browsers
Please use the latest Firefox or Chrome if possible. The HTML/JS UI uses some
recent JavaScript features which might not be supported by older browsers.

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

## Developing the frontend
The HTML/JS frontend uses [Webpack](https://webpack.github.io/) as the build system. You will need [NodeJS](https://nodejs.org/en/) and NPM (the package manager) to build the frontend pages. It does jobs like bundling the JavaScript modules together, minifying and transpiling the [ES2015](https://babeljs.io/learn-es2015/) code for older browsers, compiling [Riot tags](http://riotjs.com/), minifying/autoprefixing CSS etc.

After you've cloned this repo, run `npm install` in the root folder to install the package dependencies. There are two commands specified in `package.json` which run Webpack:

* `npm run serve` start a web server which serves the static HTML files in the `html` directory. It also watches for changes in the source files in `html-src` and automatically rebuilds and reloads them. Use this while hacking on the frontend.
* `npm run build` produces a production build of the frontend JS/CSS bundles. Use this before uploading your work to the ESP8266.

## Acknowledgements

TODO

## License
The project is licensed under GPLv3.

