#include <iostream>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/marker.h>
#include <aruco/cvdrawingutils.h>

using namespace aruco;

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
    int n_markers = 50;
    
    aruco::MarkerDetector MDetector;
    std::vector<cv::Mat>* marker_images;
    marker_images = new std::vector<cv::Mat>(n_markers);
    //aruco::Dictionary marker_dict = getPredefinedDictionary(DICT_6X6_50);
    //aruco::Dictionary marker_dict = aruco::Dictionary::loadPredefined("ARUCO_MIP_36h12");
    //float MarkerSize = 0.5;
    
    //No need to draw images for ARUCO_MIP_36h12 - downloaded
    /*if(init != true)
    {
        for(int i = 0; i < n_markers; i++)
        {
            std::cout << "Saving marker : " << i << std::endl;
            cv::Mat marker_image = marker_images->at(i);
            marker_dict
            aruco::drawMarker(marker_dict,i,200,marker_image);
            std::string file_name = boost::lexical_cast<std::string>(i) + ".jpg";
            std::string file_location = marker_directory + "/" + file_name;
            std::cout << "file location : " << file_location;
            while(!cv::imwrite(file_location,marker_image))
            {
                continue;
            }
        }
        init = true;
    }*/
    
    //Capturing camera input
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    
    //Camera calibration
    cv::Mat camera_matrix, dist_coeffs;
    std::string calibration_file_ = "/home/yeshi/projects/aruco_markers/out_camera_data.yml";
    cv::FileStorage fs;
    fs.open(calibration_file_,cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();
    
    aruco::CameraParameters camera_parameters;
    camera_parameters.readFromXMLFile(calibration_file_);
    
    //Marker detection and pose estimation
    /*bool loop = true;
    std::vector<cv::Vec3d> rvecs, tvecs;
    std::vector<int> markersIds;
    std::vector< std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
    DetectorParameters parameters;
    while(loop)
    {
        cv::Mat inputImage;
        inputVideo.read(inputImage);
        
        detectMarkers(inputImage, marker_dict, markerCorners, markersIds, parameters, rejectedCandidates);
        drawDetectedMarkers(inputImage,markerCorners,markersIds);
        estimatePoseSingleMarkers(markerCorners,0.05,camera_matrix,dist_coeffs,rvecs,tvecs);
        
        for(int i=0; i < markersIds.size(); i++)
        {
            //Pose values of each marker
            drawAxis(inputImage,camera_matrix,dist_coeffs,rvecs[i],tvecs[i],0.1);
        }
        cv::namedWindow("Input Image",CV_WINDOW_AUTOSIZE);
        cv::imshow("Input Image",inputImage);
        char key = (char) cv::waitKey(1);
        if (key == 27)
        {
            std::cout << "Esc key pressed!" << std::endl;
            break; 
        }
    }*/
    //MDetector.setThresholdParams(7,7);
    //MDetector.setThresholdParamRange(2,0);
    MDetector.setDictionary("ARUCO_MIP_36h12");
    std::map<uint32_t,aruco::MarkerPoseTracker> MTracker;
    cv::Mat inputImage;
    while(1)
    {
        inputVideo.read(inputImage);
        std::vector<aruco::Marker> markers = MDetector.detect(inputImage,camera_parameters,1,false);
        //inputImage = MDetector.getThresholdedImage(); //This works
        //std::cout << "Detected markers : " << markers.size() << std::endl;
        for(int i=0; i < markers.size(); i++)
        {
            MTracker[i].estimatePose(markers.at(i),camera_parameters,-1);
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
    
    
    
    return 0;
}
