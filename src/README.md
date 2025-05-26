# Control software
====

You can run our control software by running the following commands in a linux terminal on a raspberry pi

## Clone repo
```bash
git clone https://github.com/D0037/WRO25_FE_UgraloViziBoci
```

## Python virtual environment
```bash
cd WRO25_FE_UgraloViziBoci/src

# Initialize python virtual environment
python3 -m venv .

# Active the virtual environment
source bin/activate
```

## Install pybind11

```bash
sudo apt update
sudo apt install python3-pybind11
```

## Setup hardware PWM
1. Edit `/boot/firmware/config.txt`
2. Add line `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`
3. Reboot

## Install python dependecies
```bash
python3 -m pip install -r requirements.txt
```

## Build C++ source files
```bash
mkdir src/bin
c++ -O3 -Wall -shared -std=c++20 -fPIC $(python3 -m pybind11 --includes) src/utils/gyro.cpp -o src/utils/bin/gyro$(python3-config --extension-suffix)
```

## Run the code
```bash
python3 main.py
```