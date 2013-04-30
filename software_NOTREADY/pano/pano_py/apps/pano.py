#! /usr/bin/env python
import roslib; roslib.load_manifest('pano_py')
import rospy
import os
import pano_opencv as cv
from pano_opencv import Mat,Size
import pano_py as pano

K = Mat(3,3,cv.CV_32FC1)
K.fromarray([620.2,0,320,0,620.2,240,0,0,1])
D = Mat()
camera = pano.Camera()
camera.setCameraIntrinsics(K,D,Size(640,480))
camera.write("camera.yml")

options = pano.Options()
options.directory = "/temp/pano"
options.image_names.assign(["image.1.jpg","image.2"])

params = options.fitter_params
params.error_thresh = 6;
params.inliers_thresh = 15;
params.maxiters = 100;
params.nNeeded = 2;

options.stitch_name = "/temp/pano/stitched.jpg"