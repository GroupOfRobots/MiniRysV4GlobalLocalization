#include "FlyCapture2.h"
#include <iostream>

using namespace FlyCapture2;

int main()
{
    Error error;
    Camera camera;
    CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect( 0 );
    printf("%s\n", error.GetDescription());
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;     
        return false;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    printf("%s\n", error.GetDescription());
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;     
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " " 
              << camInfo.serialNumber << std::endl;
    
    error = camera.StartCapture();
    printf("%s\n", error.GetDescription());
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;     
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;     
        return false;
    } 

    // capture loop
    for(int i = 0; i < 10; i++)
    {
        // Get the image
        Image rawImage;
        Error error = camera.RetrieveBuffer( &rawImage );
        printf("%s\n", error.GetDescription());
        if ( error != PGRERROR_OK )
        {
                std::cout << "capture error" << std::endl;
                continue;
        }

        Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
 
        char filename [18];
        sprintf(filename,"saved_image_%d.jpg", i);
        error = rgbImage.Save(filename, ImageFileFormat::JPEG);
        printf("%s\n", error.GetDescription());      
    }

    error = camera.StopCapture();
    printf("%s\n", error.GetDescription());
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }  

    camera.Disconnect();

    return 0;
}