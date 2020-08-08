part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Calibration:

Use [intrinsic_calib.cc](https://github.com/dvorak0/camera_model/blob/master/src/intrinsic_calib.cc) to calibrate your camera.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface: 

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)


# examples

## mei model
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model mei --camera-name fisheye_mei -v


## pinhole model
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model pinhole --camera-name fisheye_pinhole -v


## kannala-brandt
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model kannala-brandt --camera-name fisheye_kannala-brandt -v

## scaramuzza
./Calibration -w 9 -h 6 -i ../data/fisheye -p left -e jpg --camera-model scaramuzza --camera-name fisheye_scaramuzza -v