# dyno
> Data acquisition tool for loggging all dyno data in a single CSV file.

### Building & Running
- `cmake CMakeLists.txt`
- `make`
- `./dyno`

### Options
- You will need to configure which serial port to connect to. Currently this is done by changing the `TEMP_TTY` `#define` statement at the top of `main.cpp`. We can make this better later...
- You can change the thermocouple polling interval (in seconds) by changing `TEMP_POLL_SEC`

### Output
- This tool outputs a CSV file named `dynologger-<datetime>.csv`.
- This file is output in WinDarab CSV format. You can use the ASCII import feature of WinDarab to convert this file to a `.bmsbin` file and then open that in WinDarab for analysis.
- If you want, you can also just open it up in Excel or something (but don't to that)
