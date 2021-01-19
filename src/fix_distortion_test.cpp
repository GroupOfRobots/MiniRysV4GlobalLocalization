#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco.h"
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
	cv::Mat InImage = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);
	// cv::Mat MidImage;
	// InImage.convertTo(MidImage, CV_16SC2);
	cv::Mat OutImage;

	aruco::CameraParameters CamParam;
	if (argc > 1) {
		CamParam.readFromXMLFile(argv[1]);
	}

    // cv::fisheye::undistortImage(InImage, OutImage, CamParam.CameraMatrix, CamParam.Distorsion.colRange(0, 4));
    cv::undistort(InImage, OutImage, CamParam.CameraMatrix, CamParam.Distorsion.colRange(0, 4), CamParam.CameraMatrix);
	// show image
	cv::imshow("distorted_image",InImage);
	cv::imshow("undistorted_image",OutImage);
	cv::waitKey(0);
}