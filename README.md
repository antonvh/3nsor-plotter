# 3nsor - the BrickPi rope plotter #

This plotter is hung by two pieces of rope on a door and controlled by a BrickPi. Since BrickPi has no screen I built
a web interface with tornado. It's the BrickPi version of L3onardo.


## Requirements ##
- **Tornado** I had to install tornado manually, as described here http://www.tornadoweb.org/en/stable/. Pip wouldn't compile tornado
right, and the server became real slow.
- **Brickpy Pyhton** (Duh.)

## How to use ##

1. Clone the repo on the brickpi
2. Type `python plotter.py`
3. Surf to <yourbrickpiaddress>:9093
4. Move the plotter to the origin with the buttons/keyboard
5. Zero it all
6. Start plotting



## Interesting bits...##
- Jquery is loaded via CDN, so you need an internet connection while doing this.
- My wifi dongle went to sleep all the time, while I was working on this project. I used this to fix it: http://raspberrypi.stackexchange.com/questions/1384/how-do-i-disable-suspend-mode
As long as the server is running, there's no problem.

## Todo ##
- Make PID controller for pen up/down
- Continue Implementing file uploads for pictures and coordinates
- Get all buttons of a certain class to send commands over socket
