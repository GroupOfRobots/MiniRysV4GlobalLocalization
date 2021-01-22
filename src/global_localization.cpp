#include "rclcpp/rclcpp.hpp"
#include "minirys_global_localization/srv/get_minirys_global_localization.hpp"
#include <memory>
#include "aruco.h"
#include "FlyCapture2.h"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <string>

enum LocalizationStatus
{
	OK,
	NO_PHOTO_TAKEN,
	MAIN_MARKER_NOT_FOUND,
	BACKUP_MARKER_NOT_FOUND,
	ROBOT_MARKER_NOT_FOUND,
	BOTH_ENV_MARKERS_NOT_FOUND,
};

class GlobalLocalizationNode : public rclcpp::Node{
	public:
		GlobalLocalizationNode(std::string params_file) : Node("minirys_global_localization"){

			// initialize markers
			this->declare_parameter("main_env_marker_id", rclcpp::ParameterValue(0));
			this->declare_parameter("main_env_marker_size", rclcpp::ParameterValue(0.0));
			this->declare_parameter("backup_env_marker_id", rclcpp::ParameterValue(0));
			this->declare_parameter("backup_env_marker_size", rclcpp::ParameterValue(0.0));
			this->declare_parameter("robot_marker_id", rclcpp::ParameterValue(0));
			this->declare_parameter("robot_marker_size", rclcpp::ParameterValue(0.0));
			mainEnvMarker = aruco::Marker(this->get_parameter("main_env_marker_id").get_value<int>());
			mainEnvMarker.ssize = this->get_parameter("main_env_marker_size").get_value<float>();
			backupEnvMarker = aruco::Marker(this->get_parameter("backup_env_marker_id").get_value<int>());
			backupEnvMarker.ssize = this->get_parameter("backup_env_marker_size").get_value<float>();
			robotMarker = aruco::Marker(this->get_parameter("robot_marker_id").get_value<int>());
			robotMarker.ssize = this->get_parameter("robot_marker_size").get_value<float>();

			// load camera parameters from file
			this->declare_parameter("camera_parameters_file", rclcpp::ParameterValue(""));
			int i;
			for (i = params_file.length()-1; i >= 0; i--){
				if (params_file[i] == '/') break;
			}
			if (i > 0){
				params_file.erase(params_file.begin()+i+1, params_file.end());
				camera_params_file = params_file + this->get_parameter("camera_parameters_file").get_value<std::string>();
			} else camera_params_file = this->get_parameter("camera_parameters_file").get_value<std::string>();
			cameraParameters.readFromXMLFile(camera_params_file);
			
			// set marker dictionary
			markerDetector.setDictionary("ARUCO_MIP_36h12", 0.f);

			// connect to camera
			cameraError = camera.Connect( 0 );
			if (cameraError != FlyCapture2::PGRERROR_OK) RCLCPP_ERROR(this->get_logger(), "%s\nFailed to connect to camera", cameraError.GetDescription());
			cameraError = camera.GetCameraInfo( &cameraInfo );
			if (cameraError != FlyCapture2::PGRERROR_OK) RCLCPP_ERROR(this->get_logger(), "%s\nFailed to get camera info from camera", cameraError.GetDescription());
			RCLCPP_INFO(this->get_logger(), "Camera information:\n\tVendor: %s\n\tModel: %s\n\tSerial No: %d", cameraInfo.vendorName, cameraInfo.modelName, cameraInfo.serialNumber);

			// turn on the camera
			cameraError = camera.StartCapture();
			if ( cameraError == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED ) RCLCPP_ERROR(this->get_logger(), "%s\nBandwidth exceeded", cameraError.GetDescription());
			else if ( cameraError != FlyCapture2::PGRERROR_OK ) RCLCPP_ERROR(this->get_logger(), "%s\nFailed to start image capture", cameraError.GetDescription());
			else RCLCPP_INFO(this->get_logger(), "Camera capture started");

			// wait for image consistency purposes - photo taken roght after the capture starts tends to be extremely bright or dim
			std::this_thread::sleep_for(std::chrono::seconds(1));

			//initialize point 0,0,0 (usefull for calculating location) and robot position point
			point0 = cv::Mat::zeros(4, 1, CV_32FC1);
			point0.at<float>(3, 0) = 1;
			robotPosition = cv::Mat::zeros(4, 1, CV_32FC1);;
			robotPosition.at<float>(3, 0) = 1;

			// locate markers
			locate_env_markers();
			locate_robot_marker();

			// initialize service
			service = this->create_service<minirys_global_localization::srv::GetMinirysGlobalLocalization>(
				"get_minirys_global_localization",
				std::bind(&GlobalLocalizationNode::get_robot_localization,
						this,
						std::placeholders::_1,
						std::placeholders::_2));
			RCLCPP_INFO(this->get_logger(), "Global localization service initialized");
		}

		~GlobalLocalizationNode(){
			// stop the camera
			RCLCPP_INFO(this->get_logger(), "Stopping the camera...");
			camera.StopCapture();
			camera.Disconnect();
		}

	private:
		rclcpp::Service<minirys_global_localization::srv::GetMinirysGlobalLocalization>::SharedPtr service;
		std::string camera_params_file;

		FlyCapture2::Camera camera;
		FlyCapture2::CameraInfo cameraInfo;
		FlyCapture2::Error cameraError;
		FlyCapture2::Image rawImage, rgbImage;

		cv::Mat inImage;
		cv::Mat backupToMainTransformation, robotToEnvTransformation;
		cv::Mat point0, robotPosition;
		cv::Vec3f robotEulerRotations;

		aruco::Marker mainEnvMarker, backupEnvMarker, robotMarker, temporaryMarker;
		aruco::MarkerDetector markerDetector;
		aruco::CameraParameters cameraParameters;

		void get_robot_localization(
					const std::shared_ptr<minirys_global_localization::srv::GetMinirysGlobalLocalization::Request> request,
					std::shared_ptr<minirys_global_localization::srv::GetMinirysGlobalLocalization::Response> response) {
			int status = locate_robot_marker();

			switch (status) {
				case LocalizationStatus::OK:
					robotToEnvTransformation = (
						mainEnvMarker.getTransformMatrix().inv()*robotMarker.getTransformMatrix() +
						backupToMainTransformation*backupEnvMarker.getTransformMatrix().inv()*robotMarker.getTransformMatrix()
					) * 0.5;
					break;
				case LocalizationStatus::MAIN_MARKER_NOT_FOUND:
					robotToEnvTransformation = backupToMainTransformation*backupEnvMarker.getTransformMatrix().inv()*robotMarker.getTransformMatrix();
					break;
				case LocalizationStatus::BACKUP_MARKER_NOT_FOUND:
					robotToEnvTransformation = mainEnvMarker.getTransformMatrix().inv()*robotMarker.getTransformMatrix();
					break;
				case LocalizationStatus::NO_PHOTO_TAKEN:
				case LocalizationStatus::ROBOT_MARKER_NOT_FOUND:
				case LocalizationStatus::BOTH_ENV_MARKERS_NOT_FOUND:
				default:
					response->x = 999999;
					response->y = 999999;
					response->alpha = 999999;
					return;
			}

			robotEulerRotations = rotationMatrixToEulerAngles(robotToEnvTransformation);
			robotPosition = robotToEnvTransformation*point0;

			response->x = robotPosition.at<float>(0, 0);
			response->y = robotPosition.at<float>(1, 0);
			response->alpha = robotEulerRotations.at<float>(2);
		}

		int take_photo(){
			// grab image from camera
			cameraError = camera.RetrieveBuffer( &rawImage );
			if ( cameraError != FlyCapture2::PGRERROR_OK )
			{
				RCLCPP_ERROR(this->get_logger(), "%s\nCapture cameraError", cameraError.GetDescription());
				return LocalizationStatus::NO_PHOTO_TAKEN;
			}

			// convert image to rgb from greyscale
			rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

			// convert to opencv Mat object
			unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
			inImage = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
			return LocalizationStatus::OK;
		}

		int locate_env_markers(){
			// take a photo with camera
			if (take_photo()) return LocalizationStatus::NO_PHOTO_TAKEN;
    		int returnValue = LocalizationStatus::OK;

			// detect main marker location
			std::vector<aruco::Marker> Markers = markerDetector.detect(inImage, cameraParameters, mainEnvMarker.ssize);
			temporaryMarker = aruco::Marker();
			for (auto m:Markers) {
				if (m.id == mainEnvMarker.id)
					temporaryMarker = m;
			}

			// check if main environment marker is valid
    		if (!temporaryMarker.isValid()){
				RCLCPP_ERROR(this->get_logger(), "Main marker was not detected in environment.");
				returnValue += LocalizationStatus::MAIN_MARKER_NOT_FOUND;
    		} else mainEnvMarker = temporaryMarker;

			// detect backup marker location
			Markers = markerDetector.detect(inImage, cameraParameters, backupEnvMarker.ssize);
			temporaryMarker = aruco::Marker();
			for (auto m:Markers) {
				if (m.id == backupEnvMarker.id)
					temporaryMarker = m;
			}

			// check if backup environment marker is valid
			if (!temporaryMarker.isValid()) {
				RCLCPP_ERROR(this->get_logger(), "Backup marker was not detected in environment.");
				returnValue += LocalizationStatus::BACKUP_MARKER_NOT_FOUND;
			} else backupEnvMarker = temporaryMarker;

			if (returnValue == 0) backupToMainTransformation = mainEnvMarker.getTransformMatrix().inv() * backupEnvMarker.getTransformMatrix();

			return returnValue;
		}

		int locate_robot_marker(){
			if (!mainEnvMarker.isValid() || !backupEnvMarker.isValid()) {
				int returnValue = locate_env_markers();
				if (returnValue != LocalizationStatus::OK) return returnValue;
			}

			// take a photo with camera
			if (take_photo()) return LocalizationStatus::NO_PHOTO_TAKEN;

			// detect main marker location
			std::vector<aruco::Marker> Markers = markerDetector.detect(inImage, cameraParameters, robotMarker.ssize);
			temporaryMarker = aruco::Marker();
			for (auto m:Markers) {
				if (m.id == robotMarker.id)
					temporaryMarker = m;
			}

			// check if robot marker is valid
			if (!temporaryMarker.isValid()){
				RCLCPP_ERROR(this->get_logger(), "Robot marker was not detected in environment.");
				return LocalizationStatus::ROBOT_MARKER_NOT_FOUND;
			} else robotMarker = temporaryMarker;

			return LocalizationStatus::OK;
		}

		cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
		{
		    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

		    bool singular = sy < 1e-6; // If

		    float x, y, z;
		    if (!singular)
		    {
		        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
		        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
		        y = atan2(-R.at<float>(2,0), sy);
		    }
		    else
		    {
		        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
		        y = atan2(-R.at<float>(2,0), sy);
		        z = 0;
		    }
		    return cv::Vec3f(x, y, z);  
		}

};

int main(int argc, char const *argv[])
{
	setbuf(stdout, nullptr);
	std::string params_file = "--params-file";
	for (int i = 0; i < argc; i++) {
		if (!params_file.compare(argv[i]) && i < argc-1){
			params_file = argv[i+1];
			break;
		}
	}
	rclcpp::init(argc, argv);
	auto GlobalLocalizationNodeObject = std::make_shared<GlobalLocalizationNode>(params_file);
	rclcpp::spin(GlobalLocalizationNodeObject);
	rclcpp::shutdown();
	return 0;
}