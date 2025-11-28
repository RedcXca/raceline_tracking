# Documentation

## Software Requirements

1. Python 3.10
2. Matplotlib
3. Numpy

## To run

```bash
python main.py ./racetracks/Montreal.csv ./racetracks/Montreal_raceline.csv

# Optional: control rendering/headless via 3rd argument
# 1  : render every frame (default if omitted)
# 10 : skip 9 out of 10 frames (faster playback)
# 0  : headless mode (no GUI; runs to completion)

# Examples
python main.py ./racetracks/Montreal.csv ./racetracks/Montreal_raceline.csv 1
python main.py ./racetracks/Montreal.csv ./racetracks/Montreal_raceline.csv 10
python main.py ./racetracks/Montreal.csv ./racetracks/Montreal_raceline.csv 0
```

## To design controller

Edit `controller.py` to write controller. Other files can be edited, but with discretion.