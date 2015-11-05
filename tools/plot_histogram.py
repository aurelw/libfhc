#!/usr/bin/python

import sys
import cv2
import numpy as np
from matplotlib import pyplot as plt

if len(sys.argv) != 2:
    print("plot_histogram.py [image_file]")
    exit(1)

filename = sys.argv[1]

img = cv2.imread(filename,cv2.CV_LOAD_IMAGE_UNCHANGED)

# Calculate histogram with mask and without mask
# Check third argument for mask
hist_full = cv2.calcHist([img],[0],None,[256],[0,256])

plt.subplot(221), plt.imshow(img, 'gray')
plt.subplot(224), plt.plot(hist_full), plt.plot(hist_full)
plt.xlim([0,256])

plt.show()
