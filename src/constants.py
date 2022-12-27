import numpy as np

# ROBOT (mm)
LENGTH_ROBOT = 0.266
WIDTH_ROBOT = 0.266

# CAMERA (pixel)
HEIGHT_CAMERA, WIDTH_CAMERA = 480, 640
CENTER_X_CAMERA = int(WIDTH_CAMERA / 2)
CENTER_Y_CAMERA = int(HEIGHT_CAMERA / 2)

HIGHER_YELLOW, LOWER_YELLOW = np.array([35, 255, 255]), np.array([25, 50, 70])