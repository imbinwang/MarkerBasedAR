#include "../include/Marker.h"
#include "../include/MarkerDetector.h"
#include "../include/CameraCalibration.h"
#include "../include/glm.h"
#include "../include/utility.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>

int main(int argc, char *argv[])
{
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

	//load obj file
	const std::string objFilePath = "data/box.obj";
	GLMmodel *objModel = glmReadOBJ(const_cast<char*>(objFilePath.c_str()));

	//initial marker detector
	MarkerDetector markerDetector(camCalib, marker5x5);

	// read camera
	cv::VideoCapture vc;
	vc.open(0);

	if( !vc.isOpened() )
	{
		printf("Cannot find the video source\n");
		return -1;
	}

	const int frameWidth = (int) vc.get(CV_CAP_PROP_FRAME_WIDTH);
	const int frameHeight = (int) vc.get(CV_CAP_PROP_FRAME_HEIGHT);
	const double frameFPS = 30.0;
	cv::Mat frame, arDrawing;

	// record the video and pose
	std::string videoPath = "data/video.avi";
	cv::VideoWriter vw;
	vw.open(videoPath, CV_FOURCC('D','I','V','X'),frameFPS, cv::Size(frameWidth, frameHeight), true);
	if( !vw.isOpened() )
	{
		printf("Cannot write the video\n");
		return -1;
	}
	else		
	{
		printf("Open the camera\n\n");
		printf("Help:\n");
		printf("\t s -- Start to record video and pose\n"
			"\t f --  Finish the recording\n"
			"\t ESC --  Quit\n");
	}

	std::string poseFile = "data/pose.txt";
	std::ofstream outPose;
	outPose.open(poseFile);
	if( !outPose.is_open() )
	{
		printf("Cannot write the pose\n");
		return -1;
	}

	// initial GUI
	char *winNameDisplay = "ARScene";
	cv::namedWindow(winNameDisplay);
	uchar key = 0;
	uchar controlKey = 0;
	for(;;)
	{
		vc >> frame;
		if( frame.empty() ) break;
		frame.copyTo( arDrawing );

		markerDetector.processFrame(frame);
		const std::vector<Transformation> &markerTrans = markerDetector.getTransformations();
		drawCoordinates(camCalib, markerTrans, arDrawing);

		std::vector<Transformation> objTrans;
		calculateObjTransformation(markerTrans, obj2markerTranslation, objTrans);
		drawObject(camCalib, objTrans, objModel, arDrawing);

		cv::imshow(winNameDisplay, arDrawing);
		key = cv::waitKey(10);

		if(key=='s') controlKey = 's';
		if(key=='f') controlKey = 'f';
		if(key==27) break;
		if (controlKey=='s')
		{
			// record video
			vw << frame;
			if((int)objTrans.size()>0)
			{
				cv::Mat rotMat = cv::Mat::eye(3,3,CV_32FC1);
				for(int i=0; i<3; ++i)
				{
					for(int j=0; j<3; ++j)
					{
						rotMat.at<float>(i,j) = objTrans[0].r().mat[i][j];
					}
				}
				std::vector<float> rotVec;
				cv::Rodrigues(rotMat, rotVec);

				// rotation vector and translation vector
				outPose << rotVec[0] <<" " << rotVec[1] <<" " << rotVec[2] <<" "
					<< objTrans[0].t().data[0] <<" "
					<< objTrans[0].t().data[1] <<" "
					<< objTrans[0].t().data[2] << std::endl;

			}else
			{
				outPose << 0 << " " << 0 << " " << 0 << " "
					<< 0 << " "
					<< 0 << " "
					<< 0 << std::endl;
			}
		}
		if (controlKey=='f')
		{
			vw.release();
			outPose.close();
		}
	}

	cv::destroyWindow(winNameDisplay);
	vc.release();
	glmDelete(objModel);

	return 0;
}