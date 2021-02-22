
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>         // in createTrainImages to create model folder

#include <unistd.h>
#include <iostream>
#include <std_msgs/String.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/vision/vpKeyPoint.h>        // vpKeyPoint
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

#include <sstream>
#include <string>

template<class T>
std::string toString(const T &value) {
    std::ostringstream os;
    os << value;
    return os.str();
}


class Detection
{
protected:
    ros::NodeHandle n_;

    std::string path_package;       // path to package mir_vision (starting in catkin_ws)
    bool verbose;

    // Camera Settings
    vpRealSense2 realsense;
    vpCameraParameters cam_color, cam_depth; 
    vpImage<vpRGBa> I_color; 
    vpImage<unsigned char> I_gray, I_depth, I_matches, I_train;
    vpImage<uint16_t> I_depth_raw; 
    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix depth_M_color;  

    double start, looptime, fps_measured;
    std::vector<double> times_vec;

    std::string detectorName;
    std::string extractorName;
    std::string matcherName;  
    std::string configurationFile;

    int maxTrainImageNumber;
    bool learnMultiplePoses;  
    
public:
    Detection(std::string name)
    {        
        ros::NodeHandle nh("~");
        nh.param<std::string>("path_package", path_package, "src/Mir200_Sim_withD435/mir_vision");
        nh.param<bool>("verbose", verbose, true);
        nh.param<int>("maxTrainImageNumber", maxTrainImageNumber, 3);
        learnMultiplePoses = maxTrainImageNumber > 1 ? true : false;
        // nh.param<std::string>("config_color", config_color, "");
        // nh.param<std::string>("config_depth", config_depth, "");
        // nh.param<std::string>("model_color", model_color, path_package + "/model/teabox/teabox.cao" );
        // nh.param<std::string>("model_depth", model_depth, "");
        // nh.param<std::string>("init_file", init_file, "");
        // nh.param<std::string>("learning_data", learning_data, "learning/data-learned.bin");  
        // nh.param<bool>("use_ogre", use_ogre, false);
        // nh.param<bool>("use_scanline", use_scanline, false);
        // nh.param<bool>("use_edges", use_edges, true);
        // nh.param<bool>("use_klt", use_klt, true);
        // nh.param<bool>("use_depth", use_depth, true);
        // nh.param<bool>("learn", learn, false);
        // nh.param<bool>("auto_init", auto_init, false);
        // nh.param<bool>("display_projection_error", display_projection_error, false);
        // std::string parentname = vpIoTools::getParent(model_color);
        // if (model_depth.empty()) { model_depth = model_color;}        
        // if (config_color.empty()) {
        //     config_color = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(model_color) + ".xml";
        // }
        // if (config_depth.empty()) {
        //     config_depth = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(model_color) + "_depth.xml";
        // }
        // if (init_file.empty()) {
        //     init_file = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(model_color) + ".init";
        // }
        // printSettings();
    }

    ~Detection(void) {}

    bool startCamera(int width, int height, int fps)
    {   
        // *** Configuration
        rs2::config config;        
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        depth_M_color.buildFrom(vpTranslationVector(-0.015,0,0), vpQuaternionVector(0,0,0,1));
        // *** Try to open Camera
        try { realsense.open(config);}
        catch (const vpException &e) {
            ROS_INFO("Catch an exception: %s", e.what());
            ROS_INFO("Check if the Realsense camera is connected...");
            return false;}
        // *** Get and print Parameters
        cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion); //perspectiveProjWithDistortion
        cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);
        if (verbose){
            ROS_INFO("Sensor internal camera parameters for color camera: ");
            ROS_INFO("  px = %f \t py = %f", cam_color.get_px(), cam_color.get_py());
            ROS_INFO("  u0 = %f \t v0 = %f", cam_color.get_u0(), cam_color.get_v0());
            ROS_INFO("  kud = %f \t kdu = %f", cam_color.get_kud(), cam_color.get_kdu());
            ROS_INFO("Sensor internal camera parameters for depth camera: " );
            ROS_INFO("  px = %f \t py = %f", cam_depth.get_px(), cam_depth.get_py());
            ROS_INFO("  u0 = %f \t v0 = %f", cam_depth.get_u0(), cam_depth.get_v0());
            ROS_INFO("  kud = %f \t kdu = %f", cam_depth.get_kud(), cam_depth.get_kdu());
        }        
        // *** Resize images
        I_color.resize(height, width);
        I_gray.resize(height, width);
        I_depth.resize(height, width);
        I_depth_raw.resize(height, width);
        I_matches.resize(height, 2*width); 
        return true;       
    }

    void setDetectionSettings(std::string name, vpKeyPoint& keypoint)
    {
        if (name == "ORB"){
            detectorName = name;    // "FAST"
            extractorName = name;
            matcherName = "BruteForce-Hamming";
            configurationFile ="detection-config.xml";
        }
        if (name == "SIFT" || "SURF"){
            detectorName = name;
            extractorName = name;
            matcherName = "FlannBased"; // "BruteForce"
            configurationFile ="detection-config-" + name + ".xml";
        }  
        keypoint.setDetector(detectorName);
        keypoint.setExtractor(extractorName);
        keypoint.setMatcher(matcherName);
        keypoint.setFilterMatchingType(vpKeyPoint::ratioDistanceThreshold);               
    }

    void learnCube(const vpImage<vpRGBa> &I, vpMbGenericTracker &tracker, vpKeyPoint &keypoint_learning, int id)
    {
        // *** Detect keypoints on frame I
        std::vector<cv::KeyPoint> trainKeyPoints;
        double elapsedTime; 
        keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
        
        // *** Get Polygons and their vertex/corners (rois)
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
        polygons = pair.first;
        roisPt = pair.second;
        // *** Calculate HomogenousMatrix and the 3D coordinates of the keypoints
        std::vector<cv::Point3f> points3f;
        vpHomogeneousMatrix cMo;
        tracker.getPose(cMo);
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam_color, trainKeyPoints, polygons, roisPt, points3f);
        // *** Save the Keypoints and their 3D coordingates in the keypoint and a file
        keypoint_learning.buildReference(I, trainKeyPoints, points3f, true, id);      
        // keypoint_learning.saveLearningData(objectnameNumber + "_learning_data.bin", true, false);
        
        // *** Display the learned Keypoints
        vpDisplay::display(I_color);
        for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
            vpDisplay::displayCross(I_color, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
        }                tracker.track(I_color);
        tracker.getPose(cMo);
        tracker.display(I_color, cMo, cam_color, vpColor::red);
        vpDisplay::displayText(I_color, 10, 10, "Learning step: keypoints are detected on visible teabox faces", vpColor::red);
    }
    
    
    void createLearnImages(bool gray, bool depth, bool keypoints, std::string filename, std::string feature = "ORB", bool replaceLastImage=false){
        startCamera(640, 480, 30);
        vpDisplayOpenCV d_c, d_g, d_d;
        d_c.init(I_color,100, 50, "Color Stream"); 
        if (gray)  {d_g.init(I_gray,100+I_color.getWidth()+10, 50, "Gray Stream"); }
        if (depth) {d_d.init(I_depth,100, 50 + I_color.getHeight()+10, "Depth Stream"); }
        
        // *** Create folder       
        std::string path = path_package + "/model/" + filename + "/";        
        boost::filesystem::create_directories(path);
        int number = 0;
        std::string objectname = path + filename;
        std::string objectnameNumber;
        std::string ext = "_color.jpeg";

        // *** Tracker and Keypoint Settings
        vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
        vpKeyPoint keypoint_learning;          
        if (keypoints) 
        {  
            bool usexml = false;
            if (vpIoTools::checkFilename(objectname + ".xml")) {
                tracker.loadConfigFile(objectname + ".xml");
                tracker.getCameraParameters(cam_color);
                usexml = true;
            }
            if (!usexml) {
                vpMe me;
                me.setMaskSize(5);
                me.setMaskNumber(180);
                me.setRange(8);
                me.setThreshold(10000);
                me.setMu1(0.5);
                me.setMu2(0.5);
                me.setSampleStep(4);
                me.setNbTotalSample(300);
                tracker.setMovingEdge(me);
                // cam.initPersProjWithoutDistortion(839, 839, 325, 243);
                tracker.setCameraParameters(cam_color);
                tracker.setAngleAppear(vpMath::rad(70));
                tracker.setAngleDisappear(vpMath::rad(80));
                tracker.setNearClippingDistance(0.1);
                tracker.setFarClippingDistance(100.0);
                tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
            }
            tracker.setOgreVisibilityTest(false);
            tracker.setDisplayFeatures(true);
            if (vpIoTools::checkFilename(objectname + ".cao"))
                tracker.loadModel(objectname + ".cao");
            else if (vpIoTools::checkFilename(objectname + ".wrl"))
                tracker.loadModel(objectname + ".wrl"); 

            // *** Keypoint settings
            setDetectionSettings(feature, keypoint_learning);
            if (usexml) { keypoint_learning.loadConfigFile(path + configurationFile); } 
        }       

        for (int i = 0; i < maxTrainImageNumber; i++) 
        {
            ROS_INFO("i ist: %i", i);
            while (true) 
            {            
                realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);   
                
                vpDisplay::display(I_color);
                vpDisplay::displayText(I_color, 20, 20, "Click to save current image with index "+ toString(i), vpColor::red);
                vpDisplay::flush(I_color);
                if (vpDisplay::getClick(I_color, false)) {
                    break;
                }
                
                if (gray) {            
                    vpImageConvert::convert(I_color, I_gray);
                    vpDisplay::display(I_gray);
                    vpDisplay::displayText(I_gray, 20, 20, "Click to save current image.", vpColor::red);
                    vpDisplay::flush(I_gray);
                    if (vpDisplay::getClick(I_gray, false)) {
                        break;
                    }
                }            
                if(depth) {
                    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
                    vpDisplay::display(I_depth);
                    vpDisplay::displayText(I_depth, 20, 20, "Click to save current image.", vpColor::red);
                    vpDisplay::flush(I_depth);
                    if (vpDisplay::getClick(I_depth, false)) {
                        break;
                    }
                }          
            }
        
            // *** Create Image/ Object name
            for (number = 0; number < maxTrainImageNumber; number++)
            {
                if ( !vpIoTools::checkFilename(objectname + std::to_string(number) + ext) )
                { 
                    if (replaceLastImage) {objectnameNumber = objectname + std::to_string(number-1); }
                    else {objectnameNumber = objectname + std::to_string(number);}              
                    break;
                }
            }
            
            vpImageIo::write(I_color, objectnameNumber + ext); 
            if (gray)  { vpImageIo::write(I_gray,  objectnameNumber +"_gray.jpeg"); }
            if (depth) { vpImageIo::write(I_depth, objectnameNumber +"_depth.jpeg"); }
            ROS_INFO("Image(s) saved to: %s", path.c_str() );
            d_g.close(I_gray);
            d_d.close(I_depth);

            if (keypoints) 
            {  
                tracker.initClick(I_color, objectnameNumber + ".init", true);
                learnCube(I_color, tracker, keypoint_learning, i);               
            }
        
            if (i < maxTrainImageNumber-1) {
                vpDisplay::displayText(I_color, 30, 10, toString(i+1) + "/" + toString(maxTrainImageNumber) + "done. Click to learn next pose of the object.", vpColor::red);
            } else {
                 vpDisplay::displayText(I_color, 30, 10, toString(i+1) + "/" + toString(maxTrainImageNumber) + "done. Click to finish learning.", vpColor::red);
            } 
            vpDisplay::flush(I_color);
            vpDisplay::getClick(I_color, true);        
        }
        
        keypoint_learning.saveLearningData(objectnameNumber + "_learning_data.xml", false, true);   // true -> .bin
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_learn_object");
    ros::NodeHandle n("~");

    std::string object_name;
    n.param<std::string>("object_name", object_name, "Teabox" );
    ROS_INFO("Detection node to learn object %s started", object_name.c_str());

    Detection detection("LearnObject");

    
    // detection.createLearnImages(true, true, true, "Teabox", "ORB");
    // detection.createLearnImages(true, true, true, "Teabox", "SIFT");
    detection.createLearnImages(false, false, true, object_name, "SURF");

    ros::shutdown();
    return 0;
}