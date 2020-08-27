import cv2
import numpy as np
import matplotlib.pyplot as plt
map_file = "/home/drl/ros_codes/RoboRTS/tools/map/icra.pgm"

map_cv = cv2.imread(map_file)
map_np = np.asarray(map_cv)
h, w, c = map_np.shape
map_cv = cv2.resize(map_cv,(w/2, h/2))
cv2.imwrite("/home/drl/ros_codes/RoboRTS/tools/map/icra_0.1.pgm", map_cv)
plt.imshow(map_cv)
plt.show()