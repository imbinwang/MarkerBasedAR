# Marker Based AR #

### Introduction ###

This code repository demonstrated the augmented reality by single maker. 

### Requirements ###

1. OS Platform: Windows
2. [OpenCV](http://opencv.org/): The version should be below 3.0, because the 2.X APIs are different from 3.X somehow. 
3. Before you can run the code project, you should configure the correct pathes of includes, libs and dlls for aforementioned third dependencies.

### Installation ###

1. Clone the MarkerBasedAR repository

	```
	git clone https://github.com/imbinwang/MarkerBasedAR.git
	```

2. We'll call the directory that you cloned MarkerBasedAR into MarkerBasedAR_ROOT. You should print the marker file in `MarkerBasedAR_ROOT/data` and calibrate your camera in advance. 

3. Modify the maker size and camera paramters in the file `MarkerBasedAR_ROOT/src/main.cpp`

	```
	//NOTE: before capturing video
	//      MUST MODIFY the configurations
	// marker size configuration

	// ----- for object -----
	// the z coordinates value of object origin
	// MUST change when new object model coming
	float objOriginZ = 1.5f; //this is for model box 

	// ----- for marker 1 -----//the location of object w.r.t the marker origin point
	cv::Size2f marker9x9 = cv::Size2f(9.0f, 9.0f);
	float obj2marker9x9Translation[3] = {10.5f, 4.5f, objOriginZ};


	// ----- for marker 2 -----//the location of object w.r.t the marker origin point
	cv::Size2f marker5x5 = cv::Size2f(5.3f, 5.3f);
	float obj2marker5x5Translation[3] = {9.35f, 5.65f, objOriginZ};

	//the location of object w.r.t the marker
	float *obj2markerTranslation = obj2marker5x5Translation;

	//load camera intrinsic parameters
	float fx = 840.83909392801706;
	float fy = 840.83909392801706;
	float cx = 319.5;
	float cy = 239.5;
	float distortionCoeff[5] = {-0.0073528715923502968, 1.3968282968421968, 
		0., 0.,-9.3220236317679284};
	CameraCalibration camCalib(fx,fy,cx,cy,distortionCoeff);
	```

4. Run and the recorded video and pose file will be found in `MarkerBasedAR_ROOT/data`.


