Control software
====

You can run our control software by running the following commands in a linux terminal

```bash
cd WRO25_FE_UgraloViziBoci/src

# Initialize python virtual environment
python3 -m venv .

# Active the virtual environment
source bin/activate

# Install pigpio
sudo apt install pigpio python3-pigpio
sudo systemctl enable --now pigpiod

# Install dependecies
python3 -m pip install -r requirements.txt

# Run the code
python3 main.py
```

