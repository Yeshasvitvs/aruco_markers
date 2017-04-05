#include <iostream>
#include <string.h>
#include <boost/lexical_cast.hpp>


#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

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
    
    //captuging camera input - yarp
    yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelBgr > > *image_input_port;
    yarp::sig::ImageOf< yarp::sig::PixelBgr > *input_yarp_frame;
    
    if(!yarp::os::Network::initialized())
        yarp::os::Network::init();
    
    image_input_port = new yarp::os::BufferedPort<yarp::sig::ImageOf< yarp::sig::PixelBgr > >;
    if(image_input_port->open("/aruco/camera:i"))
    {
        if(yarp::os::Network::checkNetwork())
        {
            if(!yarp::os::Network::connect("/marker_camera/gazebo_yarp_plugin/camera:o",image_input_port->getName()))
                yError() << "Cannot connect to the port /marker_camera/gazebo_yarp_plugin/camera:o";
        }
    }
    
    
    //Capturing camera input - opencv
    cv::VideoCapture inputVideo_cap;
    inputVideo_cap.open(0);
    inputVideo_cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    inputVideo_cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    //Camera calibration
    cv::Mat camera_matrix, dist_coeffs;
    
    //Values corresponding to gazebo camera plugin values
    cv::Mat CM = (cv::Mat_<double>(3,3) << 510, 0, 320, 0, 510, 240, 0, 0, 1);
    cv::Mat DM = (cv::Mat_<double>(5,1) << -0.4, -0.5, -0.3, 0.0, 0.0);
    camera_matrix = CM;
    dist_coeffs = DM;
    
    /*std::string calibration_file_ = "/home/yeshi/projects/aruco_markers/out_camera_data.yml";
    cv::FileStorage fs;
    fs.open(calibration_file_,cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();*/
    
    std::cout << "Camera matrix : " << camera_matrix << std::endl;
    std::cout << "Distortion Coeff : " << dist_coeffs << std::endl;

    
    //Marker detection and pose estimation
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Mat Rmat;
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
    //parameters.maxErroneousBitsInBorderRate = 0.2; //Relative to the total number of bits
    //parameters.errorCorrectionRate = 0.6;
    
    //Corner Refinement
    parameters.doCornerRefinement = true;
    parameters.cornerRefinementWinSize = 5;
    parameters.cornerRefinementMaxIterations = 100;
    parameters.cornerRefinementMinAccuracy = 0.1;
    
    float marker_size_in_meters = 0.05;
    float axis_length = 0.02;
    
    cv::Mat inputImage, outputImage;
    
    //Kalma Filtering variables
    cv::KalmanFilter kf;
    int n_states = 18;
    int n_measurements = 6;
    int n_inputs = 0;
    
    double dt = 0.033; //Time between n_measurements(1/FPS)
    
    std::cout << "Kalman Filtering initialization" << std::endl;
    kf.init(n_states,n_measurements,n_inputs,CV_64F);
    
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-5));  //Process noise
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-4));  //Measurement noise
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));  //Error covariance
    

    //TODO Double check this
                 /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
  
  
  // position
  kf.transitionMatrix.at<double>(0,3) = dt;
  kf.transitionMatrix.at<double>(1,4) = dt;
  kf.transitionMatrix.at<double>(2,5) = dt;
  kf.transitionMatrix.at<double>(3,6) = dt;
  kf.transitionMatrix.at<double>(4,7) = dt;
  kf.transitionMatrix.at<double>(5,8) = dt;
  kf.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  kf.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  kf.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  
  
  // orientation
  kf.transitionMatrix.at<double>(9,12) = dt;
  kf.transitionMatrix.at<double>(10,13) = dt;
  kf.transitionMatrix.at<double>(11,14) = dt;
  kf.transitionMatrix.at<double>(12,15) = dt;
  kf.transitionMatrix.at<double>(13,16) = dt;
  kf.transitionMatrix.at<double>(14,17) = dt;
  kf.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  kf.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  kf.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
  
  
       /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
  
  kf.measurementMatrix.at<double>(0,0) = 1;  // x
  kf.measurementMatrix.at<double>(1,1) = 1;  // y
  kf.measurementMatrix.at<double>(2,2) = 1;  // z
  kf.measurementMatrix.at<double>(3,9) = 1;  // roll
  kf.measurementMatrix.at<double>(4,10) = 1; // pitch
  kf.measurementMatrix.at<double>(5,11) = 1; // yaw
    
    while(1)
    {
        //Opencv capture
        //inputVideo_cap.read(inputImage);
        
        //Yarp capture
        input_yarp_frame = image_input_port->read();
        //conversion from yarp image to CV Mat
        IplImage *dummy = (IplImage*)(*input_yarp_frame).getIplImage();
        inputImage = cv::cvarrToMat(dummy);
    
        cv::aruco::detectMarkers(inputImage, marker_dict, markerCorners, markersIds, parameters, rejectedCandidates);
        if(markersIds.empty())
            std::cout << "Cannot detect markers" << std::endl;
        cv::aruco::drawDetectedMarkers(inputImage,markerCorners,markersIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners,0.05,camera_matrix,dist_coeffs,rvecs,tvecs);
        
        for(int i=0; i < markersIds.size(); i++)
        {
            //Pose values of each marker
            //std::cout << "Marker ID " << markersIds.at(i) << " : " << tvecs[i] << " , " << rvecs[i] << std::endl
            //cv::Mat rvecR;
            //cv::Rodrigues(rvecs[i],rvecR);
            //std::cout << rvecR;
            //std::cout << markersIds.at(i) << " " << tvecs[i][0] << " " << tvecs[i][1] << " " << tvecs[i][2] << " " << rvecs[i][0] << " " << rvecs[i][1] << " " << rvecs[i][2]  << std::endl;
            std::cout << markersIds.at(i) << " " << tvecs[i][0] << " " << tvecs[i][1] << " " << tvecs[i][2] << " " << rvecs[i][0] << " " << rvecs[i][1] << " " << rvecs[i][2]  << " ";
            cv::aruco::drawAxis(inputImage,camera_matrix,dist_coeffs,rvecs[i],tvecs[i],axis_length);
            
            cv::Mat measurements(6,1,CV_64F);
            
            //Getting the rotation matrix
            cv::Rodrigues(rvecs[i],Rmat);
            
            /*std::cout << "Testing cv::Rodrigues : " << rvecs[i];
            cv::Vec3d r_vec;
            cv::Rodrigues(Rmat,r_vec);
            std::cout << " " << r_vec << std::endl;*/
            
          
            
            //Getting measured translation
            cv::Mat translation_measured(3,1,CV_64F);
            translation_measured = cv::Mat(tvecs[i]);
            
            //Getting measured rotation
            cv::Mat rotation_measured = Rmat;
            
            //std::cout << "Rotation Measured : " << rotation_measured << std::endl;
            
            //Checking the rotation matrix
            cv::Mat measured_eulers(3,1,CV_64F);
            cv::Mat rotation_measured_T;
            cv::transpose(rotation_measured, rotation_measured_T);
            cv::Mat isEye = rotation_measured*rotation_measured_T;
            cv::Mat I = cv::Mat::eye(3,3, isEye.type());
            if(cv::norm(I,isEye) < 1e-6)
            {
                //Converting from rotation matrix to euler angles
                //std::cout << "Rotation matrix is good" << std::endl;
                float sy = sqrt(rotation_measured.at<double>(0,0) * rotation_measured.at<double>(0,0) +  rotation_measured.at<double>(1,0) * rotation_measured.at<double>(1,0) );
                bool singular = sy < 1e-6; // If
                
                float x, y, z;
                if (!singular)
                {
                    x = atan2(rotation_measured.at<double>(2,1) , rotation_measured.at<double>(2,2));
                    y = atan2(-rotation_measured.at<double>(2,0), sy);\
                    z = atan2(rotation_measured.at<double>(1,0), rotation_measured.at<double>(0,0));
                }
                else
                {
                    x = atan2(-rotation_measured.at<double>(1,2), rotation_measured.at<double>(1,1));
                    y = atan2(-rotation_measured.at<double>(2,0), sy);
                    z = 0;
                }
                measured_eulers.at<double>(0) = x;
                measured_eulers.at<double>(1) = y;
                measured_eulers.at<double>(2) = z;
                
            }
            else 
            {
                std::cout << "Rotation matrix is bad" << std::endl;
                return 1;
            }
            
            // Set measurement to predict
            measurements.at<double>(0) = translation_measured.at<double>(0); // x
            measurements.at<double>(1) = translation_measured.at<double>(1); // y
            measurements.at<double>(2) = translation_measured.at<double>(2); // z
            measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
            measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
            measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
            
            //Filtering
            cv::Mat predict = kf.predict();
            cv::Mat estimated = kf.correct(measurements);
            
            cv::Mat translation_estimated(3,1,CV_64F);
            cv::Mat rotation_estimated(3,3,CV_64F);
            
            // Estimated translation
            translation_estimated.at<double>(0) = estimated.at<double>(0);
            translation_estimated.at<double>(1) = estimated.at<double>(1);
            translation_estimated.at<double>(2) = estimated.at<double>(2);
    
            // Estimated euler angles
            cv::Mat eulers_estimated(3, 1, CV_64F);
            eulers_estimated.at<double>(0) = estimated.at<double>(9);
            eulers_estimated.at<double>(1) = estimated.at<double>(10);
            eulers_estimated.at<double>(2) = estimated.at<double>(11);
            
            //Converting from euler angles to Roation matrix
            cv::Mat R_x = (cv::Mat_<double>(3,3) << 1, 0, 0, 
                                                    0, cos(eulers_estimated.at<double>(0)), -sin(eulers_estimated.at<double>(0)), 
                                                    0, sin(eulers_estimated.at<double>(0)), cos(eulers_estimated.at<double>(0)));
            cv::Mat R_y = (cv::Mat_<double>(3,3) << cos(eulers_estimated.at<double>(1)), 0, sin(eulers_estimated.at<double>(1)), 
                                                    0, 1, 0, 
                                                    -sin(eulers_estimated.at<double>(1)), 0, cos(eulers_estimated.at<double>(1)));
            cv::Mat R_z = (cv::Mat_<double>(3,3) << cos(eulers_estimated.at<double>(2)), -sin(eulers_estimated.at<double>(2)), 0, 
                                                    sin(eulers_estimated.at<double>(2)), cos(eulers_estimated.at<double>(2)), 0,
                                                    0, 0, 1);
            
            rotation_estimated = R_z*R_y*R_x;
            
            //std::cout << "Rotaion Estimated : " << rotation_estimated << std::endl;
            cv::Mat rvec_estimated(3,1,CV_64F);
            cv::Rodrigues(rotation_estimated, rvec_estimated);
            
            std::cout << markersIds.at(i) << " " << translation_estimated.at<double>(0) << " " << translation_estimated.at<double>(1) << " " << translation_estimated.at<double>(2) << " "
                                          << rvec_estimated.at<double>(0) << " " << rvec_estimated.at<double>(1) << " " << rvec_estimated.at<double>(2)  << std::endl;
            
            
        } //End of marker for loop
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