#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "aruco.h"
// #include "markerdetector.h"
#include "FlyCapture2.h"
using namespace  std;

int main(int argc, char const *argv[])
{
	FlyCapture2::Error error;
	FlyCapture2::Camera camera;
	FlyCapture2::CameraInfo camInfo;

	// Connect the camera
	error = camera.Connect( 0 );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		printf("%s\n", error.GetDescription());
		std::cout << "Failed to connect to camera" << std::endl;     
		return false;
	}

	// Get the camera info and print it out
	error = camera.GetCameraInfo( &camInfo );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		printf("%s\n", error.GetDescription());
		std::cout << "Failed to get camera info from camera" << std::endl;     
		return false;
	}
	std::cout << "Camera information: "
			  << camInfo.vendorName << " "
			  << camInfo.modelName << " " 
			  << camInfo.serialNumber << std::endl;
	
	error = camera.StartCapture();
	if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
	{
		printf("%s\n", error.GetDescription());
		std::cout << "Bandwidth exceeded" << std::endl;     
		return false;
	}
	else if ( error != FlyCapture2::PGRERROR_OK )
	{
		printf("%s\n", error.GetDescription());
		std::cout << "Failed to start image capture" << std::endl;     
		return false;
	}

	// retrieve image from camera
	FlyCapture2::Image rawImage;
	error = camera.RetrieveBuffer( &rawImage );
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		printf("%s\n", error.GetDescription());
		std::cout << "capture error" << std::endl;
		return false;
	}

	// convert image to rgb from greyscale
	FlyCapture2::Image rgbImage;
	rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

	// convert to opencv Mat object
	unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
	cv::Mat InImage = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

	// set detection parameters (dictionary, camera parameters, markersize, ...) and detect markers on image
	aruco::MarkerDetector MDetector;
	MDetector.setDictionary("ARUCO_MIP_36h12", 0.f);
	aruco::CameraParameters CamParam;
	if (argc > 1) {
		CamParam.readFromXMLFile(argv[1]);
	}
	// float MarkerSize = 0.0385;
	float MarkerSize = 0.163;
	vector<aruco::Marker> Markers = MDetector.detect(InImage, CamParam, MarkerSize);

	// draw markers on image
	for(auto m:Markers){
		cout << m << endl;
		m.draw(InImage, cv::Scalar(0, 0, 255), 2);
	}

	// draw axises and cubes
    if (CamParam.isValid() && MarkerSize != -1) {
    	for (auto m:Markers) {
    		cout<< "Transform matrix "<<m.id<<endl;
    		cout<<m.getTransformMatrix()<<endl;
            aruco::CvDrawingUtils::draw3dAxis(InImage, m, CamParam);
            aruco::CvDrawingUtils::draw3dCube(InImage, m, CamParam);
    	}
    }


    float distance = 0;
    for (unsigned int i = 0; i < Markers.size() - 1; i++) {
	    distance = sqrt(pow(Markers[i].Tvec.at<float>(0,0) - Markers[i+1].Tvec.at<float>(0,0), 2) + pow(Markers[i].Tvec.at<float>(1,0) - Markers[i+1].Tvec.at<float>(1,0), 2) + pow(Markers[i].Tvec.at<float>(2,0) - Markers[i+1].Tvec.at<float>(2,0), 2));
	    cout << "Distance between markers " << Markers[i].id << " and " << Markers[i+1].id << ": " << distance << endl;
    }

	// show image
	cv::imshow("image_with_markers_detected",InImage);
	cv::waitKey(0);


	// stop the camera
	error = camera.StopCapture();
	if ( error != FlyCapture2::PGRERROR_OK )
	{
		// This may fail when the camera was removed, so don't show 
		// an error message
	}  
	camera.Disconnect();

	return 0;
}