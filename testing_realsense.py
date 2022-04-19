import cv2
import numpy as np
import cv2 as cv
import glob
import time
from tqdm import tqdm
from copy import deepcopy
import scipy.optimize as sopt
from klampt.math import se3,so3
from itertools import combinations
import pickle

from realsense import RealSenseCamera


serial_numbers = {'realsense_left':"f0220315",'realsense_right':"f0271386",'realsense_torso':"f0190400"}

cams = {'realsense_left':[],'realsense_right':[],'realsense_torso':[]}
pt_clouds = deepcopy(cams)

for i in list(serial_numbers.keys()):
    print(i)
    cams.update({i:RealSenseCamera(serial_numbers[i],'L515',None)})