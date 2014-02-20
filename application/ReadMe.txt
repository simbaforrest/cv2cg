# Application Usage #

--------------------------------------------
## Config File ##
Most of the applications need config files (e.g AprilCalib, AprilTagFinder).
Config can be loaded from a text file (.cfg) and also reset by command line
arguments. Its format is explained below.

### Format ###
A config file is internally a tree of ConfigNode. There are 3 types of such
nodes:
1. Simple:
	key=val
2. Array (nodes are separated by any of ",; \n\r\t"):
	key=[node1,node2;node3 node4
	 node5	node6]
3. Map (similarly, nodes are separated by any of ",; \n\r\t"):
	key={subkey1=node1, subkey2=node2;}

For example:
varInt=1
varDouble=12.23
varString=hello
varArray=[1, 2, 3]
varArray2=[1 2;
3,4
5 6]
varMap={key1=val1, key2=val2 key3=val3;
key4=val4, key5=[v1,v2,v3;v4,v5 v6]}
varArr=[
v1, v2 v3, v4;
v5, v6, v7, v8
v9, v10, v11 v12;
{key1=val1, key2=val2}
]

Note:
1. that comment line must be a separate line whose first non-white char is '#'
2. Array or map immediately followed by ";" means that when printing this node,
it will be printed in a short format like [abc......] or {123......}
3. characters "=[]{},; \n\r\t" are reserved for parsing control; avoid using
these characters in your config file, otherwise parsing may fail; also
characters ":@" are reserved for key_path; avoid using these as key's name

--------------------------------------------
## AprilCalib ##
Purpose: camera intrinsic calibration using a set of planar or non-planar AprilTags

### Steps ###
1. Create a calibration rig using a set of planar/non-planar AprilTags.
2. Modify the AprilCalib.cfg file.
A non-planar calibration rig with 18 AprilTags starts from id 0; and each
AprilTag center's 3D coordinates are specified in the CalibRig:tagCenters
sequentially:
	CalibRig={
		mode=3d
		start_id=0
		nTags=18
		tagCenters=[
		-8.5 11 0
		0 11 0
		8.5 11 0
		-8.5 0 0
		...
		]
	}
A planar calibration rig with 9 AprilTags starts from id 0, arranges as a
3x3 matrix like:
	0 1 2
	3 4 5
	6 7 8
and the spacings are dx (distance between Tag0 and Tag1's center) by dy (
distance between Tag0 and Tag3's center):
	CalibRig={
		mode=2d
		start_id=0
		w=3
		h=3
		dx=8.5
		dy=11
	}
3. Take n images (usually n>10).
4. Run AprilCalib (assuming AprilTags in the calibration rig are in TagFamily 36h11):
	AprilCalib url=photo://*.png tagfamiliesID=4
5. Find the outputed AprilCalib_log_*.m, in which the K and distCoeffs are the intrinsic
parameters of the camera.
6. If you want to use a different number (e.g., 3) of distCoeffs (default is 2), re-run
the AprilCalib:
	AprilCalib url=photo://*.png tagfamiliesID=4 "AprilCalibprocessor={nDistCoeffs=3}"
7. For further help, run:
	AprilCalib -h

--------------------------------------------
## AprilTagFinder ##
Purpose: find AprilTags in the image(s) and log results (raw detections, images, optimization
results)

### Steps ### 
1. To run AprilTagFinder using regular usb camera:
	AprilTagFinder url=camera://0 tagfamiliesID=4
2. To run AprilTagFinder using a set of images (all png files under current directory):
	AprilTagFinder url=photo://*.png tagfamiliesID=4
3. To run AprilTagFinder using point grey camera:
	AprilTagFinder url=pgr://0 tagfamiliesID=4
4. If running using camera, press 'h' when running, the help instructions will
be outputed in the console window 
5. For further help, run:
	AprilTagFinder -h

--------------------------------------------
## AprilTagCreator ##
TODO