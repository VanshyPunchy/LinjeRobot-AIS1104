# LinjeRobot-AIS1104
To regenerate phase 2 code after changing:
platformio.ini
add/remove libraries
change build flags

You need to run:
```bash
$ pio run -e nano_esp32 -t compiledb
```
