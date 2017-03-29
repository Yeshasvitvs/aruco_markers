#include <iostream>
#include <string.h>
#include <boost/lexical_cast.hpp>


#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

//#include <opencv2/opencv.hpp>
//#include <opencv2/aruco.hpp>
//#include <opencv2/aruco/charuco.hpp>
//#include <opencv2/aruco/dictionary.hpp>

int main(int argc, char **argv) {
    
    std::string marker_directory;
    char cwd[1024];
    if(getcwd(cwd,sizeof(cwd)) != NULL)
    {
        marker_directory = cwd;
        marker_directory.replace(marker_directory.find("build"),5,"markers");
        std::cout << "marker directory : " << marker_directory << std::endl;
    }
    else std::cerr << "getcwd() error!" << std::endl;
    
    
    //Marker generation
    bool init = false;
    int n_markers = 10;
    int marker_size = 5;; 
    
    std::vector<cv::Mat>* marker_images;
    marker_images = new std::vector<cv::Mat>(n_markers);
    cv::aruco::Dictionary marker_dict = cv::aruco::generateCustomDictionary(n_markers,marker_size);
    float markerBorder_bits = 1;
    int marker_pixel_resol = 300; 
    if(init != true)
    {
        for(int i = 0; i < n_markers; i++)
        {
            //std::cout << "Saving marker : " << i << std::endl;
            cv::Mat marker_image = marker_images->at(i);
            cv::aruco::drawMarker(marker_dict,i,marker_pixel_resol,marker_image,markerBorder_bits);
            std::string file_name = boost::lexical_cast<std::string>(i) + ".jpg";
            std::string file_location = marker_directory + "/" + file_name;
            std::cout << "file location : " << file_location << std::endl;
            while(!cv::imwrite(file_location,marker_image))
            {
                continue;
            }
        }
        init = true;
    }
    
    //Capturing camera input
    cv::VideoCapture inputVideo_cap;
    inputVideo_cap.open(0);
    inputVideo_cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    inputVideo_cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    //Camera calibration
    cv::Mat camera_matrix, dist_coeffs;
    std::string calibration_file_ = "/home/yeshi/projects/aruco_markers/out_camera_data.yml";
    cv::FileStorage fs;
    fs.open(calibration_file_,cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();

    
    //Marker detection and pose estimation
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> markersIds;
    std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters parameters;
    
    //Thresholding parameters
    /*parameters.adaptiveThreshWinSizeMin = 10;
    parameters.adaptiveThreshWinSizeMax = 30;
    parameters.adaptiveThreshWinSizeStep = 3;
    parameters.adaptiveThreshConstant = 7;*/
    
    //Contour Filtering parameters
    /*parameters.minMarkerPerimeterRate = 30/inputVideo_cap.get(CV_CAP_PROP_FRAME_WIDTH); //Specified relative to the max dimension of input image 640x0.05 = 32 pixels
    parameters.maxMarkerPerimeterRate = 60/inputVideo_cap.get(CV_CAP_PROP_FRAME_WIDTH);
    
    parameters.minCornerDistanceRate = 0.05;
    parameters.minMarkerDistanceRate = 0.05;
    parameters.minDistanceToBorder = 3;*/
   
    //Bits extraction
    /*parameters.markerBorderBits = markerBorder_bits;
    parameters.minOtsuStdDev = 2;
    parameters.perspectiveRemovePixelPerCell = 10;
    parameters.perspectiveRemoveIgnoredMarginPerCell = 0.2;*/
    
    //Marker Identification
    parameters.maxErroneousBitsInBorderRate = 0.2; //Relative to the total number of bits
    parameters.errorCorrectionRate = 0.6;
    
    //Corner Refinement
    parameters.doCornerRefinement = true;
    parameters.cornerRefinementWinSize = 5;
    parameters.cornerRefinementMaxIterations = 100;
    parameters.cornerRefinementMinAccuracy = 0.1;
    
    float marker_size_in_meters = 0.05;
    float axis_length = 0.02;
    
    cv::Mat inputImage, outputImage;
    
    
    while(1)
    {
        
        inputVideo_cap.read(inputImage);
        
        
        cv::aruco::detectMarkers(inputImage, marker_dict, markerCorners, markersIds, parameters, rejectedCandidates);
        if(markersIds.empty())
            std::cout << "Cannot detect markers" << std::endl;
        cv::aruco::drawDetectedMarkers(inputImage,markerCorners,markersIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners,0.05,camera_matrix,dist_coeffs,rvecs,tvecs);
        
        for(int i=0; i < markersIds.size(); i++)
        {
            //Pose values of each marker
            cv::aruco::drawAxis(inputImage,camera_matrix,dist_coeffs,rvecs[i],tvecs[i],axis_length);
        }
        cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
        cv::imshow("Input Image",inputImage);
        char key = (char) cv::waitKey(1);
        if (key == 27)
        {
            std::cout << "Esc key pressed!" << std::endl;
            break; 
        }
        markersIds.clear();
        markerCorners.clear();
    }
    
    return 0;
}

//Implementaion using only aruco library without
/*
 * 
 * #include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/marker.h>
#include <aruco/cvdrawingutils.h>

using namespace aruco;
aruco::MarkerDetector MDetector;
MDetector.setThresholdParams(7,7);
MDetector.setThresholdParamRange(2,0);
MDetector.setDictionary("ARUCO_MIP_36h12");

//aruco::Dictionary marker_dict = aruco::Dictionary::loadPredefined("ARUCO_MIP_36h12");

std::string calibration_file_ = "/home/yeshi/projects/aruco_markers/out_camera_data.yml";
aruco::CameraParameters camera_parameters;
camera_parameters.readFromXMLFile(calibration_file_);
std::cout << camera_parameters.CamSize << std::endl;
std::cout << camera_parameters.CameraMatrix << std::endl;
std::cout << camera_parameters.Distorsion << std::endl;

std::map<uint32_t,aruco::MarkerPoseTracker> MTracker;
    cv::Mat inputImage;
    while(1)
    {
        inputVideo.read(inputImage);
        std::vector<aruco::Marker> markers = MDetector.detect(inputImage);
        //inputImage = MDetector.getThresholdedImage(); //This works
        //std::cout << "Detected markers : " << markers.size() << std::endl;
        for(int i=0; i < markers.size(); i++)
        {
            MTracker[i].estimatePose(markers.at(i),camera_parameters,-1,4);
            //markers[i].draw(inputImage,cv::Scalar(0,0,255),2);
            aruco::CvDrawingUtils::draw3dAxis(inputImage,markers[i],camera_parameters);
        }
        cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
        cv::imshow("Input Image",inputImage);
        char key = (char) cv::waitKey(1);
        if (key == 27)
        {
            std::cout << "Esc key pressed!" << std::endl;
            break; 
        }
        
    }

*/