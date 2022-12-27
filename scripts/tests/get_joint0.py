import numpy as np

HEIGHT, WIDTH = 480, 640

LONGUEUR = 281
LARGUEUR = 306

CENTER_X_BALL= 177

oppose = (CENTER_X_BALL * LARGUEUR) / WIDTH

offset_tangent = LONGUEUR / 2
tangent = LONGUEUR / 2 + offset_tangent

print(oppose, tangent)

joints0 = np.arctan2(oppose, tangent)

print(joints0)  