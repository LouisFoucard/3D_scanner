# 3D_scanner
blender python script to receive and process data from an arduino/lidar/mpu6050 based 3D_scanner. Instead of relying on the 
reading servo motors positionl, and IMU (mpu 6050) is used to determine the quaternion that defines the lidar orientation.
This reduces the error made in reconstrcuting the 3d position of the points, and makes the scanner more robust: if the scanner is moved midway through the scan, the IMU will track that additional rotation, and the scan will take that into account.

Here is a short description of the different files:

_3D_scanner.ino:  C++ arduino driver, takes care of data gathering from the sensors, and rotates the servos to complete the scan. Also sends the sensor data to the python script 3D_scanner.ipynb.

3D_scanner.ipynb: interfaces between the scanner and Blender, a 3D simulation software used to render the scan. Values from the sensors are converted into 3d coordinates and are passed to Blender. (futur version will skip this step all together and go directly to Blender from Arduino)

3D_scanner.blend and blender_python_script.py: Blender file and a blender python script which receives the sensor data, parses through the points coordinates and creates a NURB interpolating surface that represent the scanned surface. Points are stored in such a way that the surface reconstruction is extremely fast and can be done in real time. 
Below is an example of a reconstructed 3d cloud point, and the resuling interpolated surface:

![alt tag](https://github.com/LouisFoucard/3D_scanner/blob/master/blender_screen.png)

Here are the schematics of the scanner:

![alt tag](https://github.com/LouisFoucard/3D_scanner/blob/master/3D_scanner_fritzing_bb.png)

