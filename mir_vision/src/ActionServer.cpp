// #define VISP_HAVE_REALSENSE2 true
// #define VISP_HAVE_OPENCV true
// #if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OPENCV)

#include <mir_vision/CamDetectionAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "nav_msgs/Odometry.h"             // Subscribe to Odom
#include <boost/filesystem.hpp>         // in createTrainImages to create model folder

#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>                      // for saving files 
#include <math.h>                       // for pi

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



#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef actionlib::SimpleActionServer<mir_vision::CamDetectionAction> Server; 

class CamDetectionAction
{
protected:

    ros::NodeHandle n_;
    actionlib::SimpleActionServer<mir_vision::CamDetectionAction> server_;    
    mir_vision::CamDetectionResult result_;
    int STATE_SEARCHING = 0;
    int STATE_FINISH = 1;
    int STATE_FOUND_MATCH = 2;
    int STATE_REFINE = 3;

    std::string path_package;       // path to package mir_vision (starting in catkin_ws)
    bool verbose;

    // Camera Settings
    vpRealSense2 realsense;
    vpCameraParameters cam_color, cam_depth; 
    vpImage<vpRGBa> I_color; 
    vpImage<unsigned char> I_gray, I_depth, I_matches, I_train;
    vpImage<uint16_t> I_depth_raw; 
    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix cMo_prev;
    vpHomogeneousMatrix cMo_diff;
    vpHomogeneousMatrix depth_M_color; 
    std::vector<vpHomogeneousMatrix> cMo_vec; 

    double start, looptime, fps_measured;
    std::vector<double> times_vec;

    std::string detectorName;
    std::string extractorName;
    std::string matcherName;  
    std::string configurationFile;

    ros::Subscriber sub_;
    bool moving;
    int frameThreshold;
    int frame;
    
public:
    CamDetectionAction(std::string name):
        server_(n_, name, boost::bind(&CamDetectionAction::executeCB, this, _1), false)
    {        
        server_.start();
        sub_ = n_.subscribe("/odom_comb", 1, &CamDetectionAction::analysisMovement, this);
        moving = false;         // In case topic isnt published
        ros::NodeHandle nh("~");
        nh.param<std::string>("path_package", path_package, "src/Mir200_Sim_withD435/mir_vision");
        nh.param<bool>("verbose", verbose, true);
        nh.param<int>("frameThreshold", frameThreshold, 20);
        cMo_vec.assign(frameThreshold, cMo);


        // nh.param<std::string>("parent_frame", parent_frame, "camera_arm_color_optical_frame");
        // nh.param<std::string>("child_frame", child_frame, "object");
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
        // nh.param<bool>("broadcast_transform", broadcast_transform, true);
        // nh.param<double>("proj_error_threshold", proj_error_threshold, 25);
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

    ~CamDetectionAction(void) {}

    // void printSettings() 
    // {
    //     std::cout << "Tracked features: " << std::endl;
    //     std::cout << "  Use edges   : " << use_edges << std::endl;
    //     std::cout << "  Use klt     : " << use_klt << std::endl;
    //     std::cout << "  Use depth   : " << use_depth << std::endl;
    //     std::cout << "Tracker options: " << std::endl;
    //     std::cout << "  Use ogre    : " << use_ogre << std::endl;
    //     std::cout << "  Use scanline: " << use_scanline << std::endl;
    //     std::cout << "  Proj. error : " << proj_error_threshold << std::endl;
    //     std::cout << "  Display proj. error: " << display_projection_error << std::endl;
    //     std::cout << "Config files: " << std::endl;
    //     std::cout << "  Config color: " << "\"" << config_color << "\"" << std::endl;
    //     std::cout << "  Config depth: " << "\"" << config_depth << "\"" << std::endl;
    //     std::cout << "  Model color : " << "\"" << model_color << "\"" << std::endl;
    //     std::cout << "  Model depth : " << "\"" << model_depth << "\"" << std::endl;
    //     std::cout << "  Init file   : " << "\"" << init_file << "\"" << std::endl;
    //     std::cout << "Learning options   : " << std::endl;
    //     std::cout << "  Learn       : " << learn << std::endl;
    //     std::cout << "  Auto init   : " << auto_init << std::endl;
    //     std::cout << "  Learning data: " << learning_data << std::endl;        
    // }

    void analysisMovement(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double t = 0.01;   // threshold
        double x = msg->twist.twist.linear.x;
        double z = msg->twist.twist.angular.z;
        bool moving_old = moving;

        moving = (abs(x) > t || abs(z) > t) ? true : false;

        if (moving != moving_old && verbose) 
        {
            if (moving) 
                ROS_INFO("Robot starts to move");
            else
                ROS_INFO("Robot stops moving");
        }
                    
    }

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

    void executeCB(const mir_vision::CamDetectionGoalConstPtr &goal)
    {
        frame = 0;
        bool proceed = false;
        bool success = false;
        bool usexml = false;
        // cMo_prev.init();
        std::string objectName = goal->object_name;
        std::string learningData = goal->learning_data;
        bool binMode = ( learningData.rfind(".bin") != std::string::npos );

        std::string path = path_package + "/model/" + objectName + "/";
        // if (!use_edges && !use_klt && !use_depth) {
        //     std::cout << "You must choose at least one visual features between edge, KLT and depth." << std::endl;
        //     server_.setPreempted();
        //     proceed = false;
        // }
        // if (config_color.empty() || config_depth.empty() || model_color.empty() || model_depth.empty() || init_file.empty()) {
        //     std::cout << "config_color.empty() || config_depth.empty() || model_color.empty() || model_depth.empty() || init_file.empty()" << std::endl;
        //     server_.setPreempted();
        //     proceed = false;
        // }
        
        // *** Start camera and check if this worked
        int width = 640, height = 480, fps = 30;
        proceed = startCamera(width, height, fps); 
        if (!proceed) {server_.setPreempted();} 
        ROS_INFO("Start Camera");      
        // *** Read train image as reference
        // vpImageIo::read(I_train, path_package + "/model/" + goal->object_name + "/" + goal->object_name + "_gray.jpeg");
        // *** Init Displays and insert images
        vpDisplayOpenCV d_c, d_g, d_d, d_matches;
        unsigned int _posx = 100, _posy = 50;        
        d_g.init(       I_gray,  _posx,                         _posy,                          "Color stream");
        // d_d.init(       I_depth, _posx + I_gray.getWidth()+10,  _posy,                          "Depth stream");
        // d_matches.init(I_matches,_posx,                         _posy + I_gray.getHeight()+10,  "Matching Keypoints");    
        // I_matches.insert(I_train, vpImagePoint(0, 0));
        // I_matches.insert(I_gray, vpImagePoint(0, I_train.getWidth()));
        
        // *** Tracker settings
        vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
        if (vpIoTools::checkFilename(path + objectName + ".xml")) {
            tracker.loadConfigFile(path + objectName + ".xml");
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
            me.setNbTotalSample(250);
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
        if (vpIoTools::checkFilename(path + objectName + ".cao"))
            tracker.loadModel(path + objectName + ".cao");
        else if (vpIoTools::checkFilename(path + objectName + ".wrl"))
            tracker.loadModel(path + objectName + ".wrl");

        // *** Set detector, extractor and matcher
        vpKeyPoint keypoint;
        setDetectionSettings("SURF", keypoint);
        if (usexml) {
            keypoint.loadConfigFile(path + configurationFile);
        } else {            
            keypoint.setMatchingRatioThreshold(0.8);
            keypoint.setUseRansacVVS(true);
            keypoint.setUseRansacConsensusPercentage(true);
            keypoint.setRansacConsensusPercentage(20.0);
            keypoint.setRansacIteration(200);
            keypoint.setRansacThreshold(0.005);
        }
        
        ROS_INFO("%s", (path+learningData).c_str());
        keypoint.loadLearningData(path + learningData, binMode);      
        // ROS_INFO("Reference keypoints = %i", keypoint.buildReference(I_train) );

        mir_vision::CamDetectionFeedback feedback_previous;
        // *** Detection Loop
        while(server_.isActive() && proceed && ros::ok())
        {
            start = vpTime::measureTimeMs();
            double error;
            double elapsedTime;
            mir_vision::CamDetectionFeedback feedback_;

            // *** Check if preempt is requested
            if (server_.isPreemptRequested() )
            {
                ROS_INFO("Preempted");
                server_.setPreempted();
                proceed = false;
                break;                              // quits the while 
            } 
            
            // *** Aquire frames and display them
            realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);
            vpImageConvert::convert(I_color, I_gray);
            vpDisplay::display(I_gray);
            vpDisplay::displayText(I_gray, 10, 10, "Detection and localization in process...", vpColor::red);
            
            if (keypoint.matchPoint(I_gray, cam_color, cMo, error, elapsedTime)) {
                // ROS_INFO("Error: %f", error);
                tracker.setPose(I_gray, cMo);

                // *** Prepare Feedback
                feedback_.estimated_pose = createPosesStamped(cMo);
                feedback_.state = STATE_FOUND_MATCH;

                // POSE REFINEMENT
                // *** In case robot stops moving -> Start to calculate finale cMo
                if (!moving){                    
                    cMo_vec.push_back(cMo);
                    frame++;
                    feedback_.state = STATE_REFINE;
                    if (frame >= frameThreshold)
                    {
                        // *** Compute resulting cMo from cMo_vec                        
                        computeResult(cMo, result_.translation_stdev, result_.rotation_stdev);   
                        // *** Create feedback and result
                        feedback_.state = STATE_FINISH; // Feedback is still published after result is send
                        result_.object_pose = createPosesStamped(cMo);
                        // *** Send Result
                        ROS_INFO("Execute successfully done: Found %s!", objectName.c_str());
                        server_.setSucceeded(result_);
                        proceed = false;
                        success = true;
                    }
                } else {
                    cMo_vec.clear();
                    frame = 0;
                }
                // *** Display model and frame in image
                tracker.display(I_gray, cMo, cam_color, vpColor::red, 2);                   // Is this shown after server is set to succeed?
                vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.025, vpColor::none, 3);
                // *** Publish Feedback
                server_.publishFeedback(feedback_);
                ROS_INFO("Feedback state %i", feedback_.state );
            } else {
                feedback_.state = STATE_SEARCHING;
                if (!moving) {
                    server_.publishFeedback(feedback_);
                    ROS_INFO("Feedback state %i", feedback_.state );
                }
            } 

            // *** Send Feedback only when cMo is different to cMo_prev             
            // cMo.print(); std::cout << std::endl;
            // printHomogeneousMatrix(cMo);
            // printRPY(cMo);
            // vpMatrix M;
            // vpMatrix::sub2Matrices(cMo_prev, cMo, M);
            // double e = 0.1;
            // if (M.getMaxValue() > e){                    
            //     printMatrix(M); 
            //     ROS_INFO("Max difference is: %.3f", M.getMaxValue());
            //     sendFeedback(cMo);
            // }                
            // cMo_prev = cMo;

            // *** Print Feedback
            // if (feedback_.state != feedback_previous.state) {ROS_INFO("Feedback state switched to %i", feedback_.state );}              
            // feedback_previous = feedback_;

            vpDisplay::flush(I_gray);
            
            // I_matches.insert(I_gray, vpImagePoint(0, I_train.getWidth()));
            // vpDisplay::display(I_matches);
            // vpDisplay::displayLine(I_matches, vpImagePoint(0, I_train.getWidth()), 
            //             vpImagePoint(I_train.getHeight(), I_train.getWidth()), vpColor::white, 2);              
            // if (use_depth) {
            //     vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
            //     vpDisplay::display(I_depth);
            //     vpDisplay::flush(I_depth);
            // }
            // unsigned int nbMatch = keypoint.matchPoint(I_gray);     // I_gray is matched with reference image
            // ROS_INFO("Matches = %i", nbMatch);
            // vpImagePoint iPref, iPcur;
            // for (unsigned int i = 0; i < nbMatch; i++) {
            //     keypoint.getMatchedPoints(i, iPref, iPcur);
            //     vpDisplay::displayLine(I_matches, iPref, iPcur + vpImagePoint(0, I_train.getWidth()), vpColor::green);
            // }
            // vpDisplay::flush(I_matches);

            looptime = vpTime::measureTimeMs() - start;
            times_vec.push_back(looptime);
            fps_measured = 1/looptime;
        }  

        ROS_INFO("Execute finished!");
        if (!times_vec.empty() && verbose) 
        {
        std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
                << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;
        } 
        // saveData();

        while(ros::ok())
        {
            realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);
            vpImageConvert::convert(I_color, I_gray);
            vpDisplay::display(I_gray);
            vpDisplay::displayText(I_gray, 10, 10, "Click to close camera and finsih goal execution", vpColor::red);
            if (success) {
                tracker.display(I_gray, cMo, cam_color, vpColor::red, 2);                   
                vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.025, vpColor::none, 3);
            }

            if (vpDisplay::getClick(I_gray, false))
                break;
        }
    }

    void saveData()
    {
        std::string filename = path_package + "/DetectionDataLog.txt";
        std::ofstream f;
        f.open(filename.c_str());

        f << "cMo_vec" << std::endl;
        for (int i = 0; i < cMo_vec.size(); i++)
        {
            f << cMo_vec[i] << ";\n" << std::endl;
        }

        f << "\nCalculated cMo" << std::endl;
        f << cMo << std::endl;

        f << "\nresult object pose" << std::endl;
        f << result_.object_pose << std::endl;           

        f.close();
    }

    void printHomogeneousMatrix(const vpHomogeneousMatrix M)
    {
        int precision = 3;
        std::stringstream s0, s1, s2, s3;
        s0.precision(precision);
        s1.precision(precision);
        s3.precision(precision);
        s0.precision(precision);

        s0 << "[" << M[0][0] << "; " << M[0][1] << "; " <<M[0][2]<< "; " << M[0][3] << "]";
        s1 << "[" << M[1][0] << "; " << M[1][1] << "; " <<M[1][2]<< "; " << M[1][3] << "]";
        s2 << "[" << M[2][0] << "; " << M[2][1] << "; " <<M[2][2]<< "; " << M[2][3] << "]";
        s3 << "[" << M[3][0] << "; " << M[3][1] << "; " <<M[3][2]<< "; " << M[3][3] << "]";
        ROS_INFO("\n %s\n %s\n %s\n %s\n", s0.str().c_str(), s1.str().c_str(), s3.str().c_str(), s3.str().c_str());

        // std::string s1;
        // s1 = "[%.2f; %.2f; %.2f; %.2f]", M[0][0], M[0][1], M[0][2], M[1][3];
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[0][0], M[0][1], M[0][2], M[0][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[1][0], M[1][1], M[1][2], M[1][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[2][0], M[2][1], M[2][2], M[2][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[3][0], M[3][1], M[3][2], M[3][3] );   
    }
    
    void printMatrix(const vpMatrix& M)
    {
        int precision = 3;
        std::stringstream s0, s1, s2, s3;
        s0.precision(precision);
        s1.precision(precision);
        s3.precision(precision);
        s0.precision(precision);

        s0 << "[" << M[0][0] << "; " << M[0][1] << "; " <<M[0][2]<< "; " << M[0][3] << "]";
        s1 << "[" << M[1][0] << "; " << M[1][1] << "; " <<M[1][2]<< "; " << M[1][3] << "]";
        s2 << "[" << M[2][0] << "; " << M[2][1] << "; " <<M[2][2]<< "; " << M[2][3] << "]";
        s3 << "[" << M[3][0] << "; " << M[3][1] << "; " <<M[3][2]<< "; " << M[3][3] << "]";
        ROS_INFO("\n %s\n %s\n %s\n %s\n", s0.str().c_str(), s1.str().c_str(), s3.str().c_str(), s3.str().c_str());

        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[0][0], M[0][1], M[0][2], M[0][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[1][0], M[1][1], M[1][2], M[1][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[2][0], M[2][1], M[2][2], M[2][3] );
        // ROS_INFO("[%.2f; %.2f; %.2f; %.2f]", M[3][0], M[3][1], M[3][2], M[3][3] );   
    }
    
    void computeResult(vpHomogeneousMatrix& M, geometry_msgs::Point t_stdev, geometry_msgs::Point r_stdev )
    {
        std::vector<double> px_vec, py_vec, pz_vec, rotx_vec, roty_vec, rotz_vec;
        double rotx, roty, rotz;
        Eigen::Vector3d euler_angles;
        vpRotationMatrix R;

        for (int i = 0; i < cMo_vec.size(); i++)
        {
            px_vec.push_back(cMo_vec[i][0][3]);
            py_vec.push_back(cMo_vec[i][1][3]);
            pz_vec.push_back(cMo_vec[i][2][3]);

            euler_angles = getRPY(cMo_vec[i], false, false);
            rotz_vec.push_back(euler_angles[0]);    // gier = yaw
            roty_vec.push_back(euler_angles[1]);    // nick = pitch
            rotx_vec.push_back(euler_angles[2]);    // roll
        }  

        // *** Mean     
        M[0][3] = vpMath::getMean(px_vec);
        M[1][3] = vpMath::getMean(py_vec);
        M[2][3] = vpMath::getMean(pz_vec);
        rotz    = vpMath::getMean(rotz_vec);    
        roty    = vpMath::getMean(roty_vec);    
        rotx    = vpMath::getMean(rotx_vec);
        R.buildFrom(vpRzyxVector(rotz, roty, rotx)); 
        M.insert(R);       
        
        // Stdev
        t_stdev.x = vpMath::getStdev(px_vec);
        t_stdev.y = vpMath::getStdev(py_vec);
        t_stdev.z = vpMath::getStdev(pz_vec);
        r_stdev.x = vpMath::getStdev(rotx_vec);
        r_stdev.y = vpMath::getStdev(roty_vec);
        r_stdev.z = vpMath::getStdev(rotz_vec);

    } 

    Eigen::Vector3d getRPY(const vpHomogeneousMatrix& M, bool output=false, bool degree=true) 
    {
        // *** Get Rotation
        vpRotationMatrix R;
        M.extract(R);

        Eigen::Matrix3d rotation_matrix;
        for (unsigned int ii = 0; ii < R.getRows(); ii++) {
            for (unsigned int jj = 0; jj < R.getCols(); jj++) {
                rotation_matrix(ii, jj) = R[ii][jj];
            }
        }
        // 2.1 Converting the rotation matrix to Euler angle
        // ZYX order intrinsic -> extrinsic (unitXYZ): roll around x axis, then around y axis pitch, finally around z axis yaw, 
        // 0 for X axis, 1 for Y axis, 2 for Z axis
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);    
        
        if (degree)
        {
            // *** Convert to degree
            euler_angles[0] = euler_angles[0] *180/M_PI;
            euler_angles[1] = euler_angles[1] *180/M_PI;
            euler_angles[2] = euler_angles[2] *180/M_PI;
        }

        if (output) 
        {
            std::stringstream ss; 
            ss << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose();
            ROS_INFO("%s", ss.str().c_str());
        }        
        return euler_angles;
    }   
    
    geometry_msgs::PoseStamped createPosesStamped(const vpHomogeneousMatrix& M)
    {        
        vpQuaternionVector q;
        vpTranslationVector translation;
        geometry_msgs::PoseStamped poseStamped; 
        
        M.extract(q);
        M.extract(translation);

        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "camera_arm_color_optical_frame";

        poseStamped.pose.position.x = translation[0];
        poseStamped.pose.position.y = translation[1];
        poseStamped.pose.position.z = translation[2];
        poseStamped.pose.orientation.x = q.x();
        poseStamped.pose.orientation.y = q.y();
        poseStamped.pose.orientation.z = q.z();
        poseStamped.pose.orientation.w = q.w();
        // broadcast_transformation(parent_frame, child_frame, q, translation);
        return poseStamped;
    }


    
    std::string parent_frame, child_frame;
    std::string config_color, config_depth;
    std::string model_color, model_depth;
    std::string init_file;
    std::string learning_data;
    bool use_ogre;
    bool use_scanline;
    bool use_edges;
    bool use_klt;
    bool use_depth;
    bool learn;
    bool auto_init;
    bool display_projection_error;
    bool broadcast_transform;
    double proj_error_threshold;
 
};

void broadcast_transformation(const std::string parent_frame, const std::string child_frame, const vpQuaternionVector& q, const vpTranslationVector& translation) 
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;

  transformStamped.transform.translation.x = translation[0];
  transformStamped.transform.translation.y = translation[1];
  transformStamped.transform.translation.z = translation[2];
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}


// void execute(const mir_vision::CamDetectionGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended here
// {    
//     q[0] = 0;
//     q[1] = 1;
//     q[2] = 0;
//     q[3] = 0;      
//     // if (broadcast_transform) {
//     //     ros::init(argc, argv, "cam_object_broadcaster");
//     // }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  TRACKER settings 
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     std::vector<int> trackerTypes;
//     if (use_edges && use_klt)
//         trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
//     else if (use_edges)
//         trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER );
//     else if (use_klt)
//         trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER);
//     if (use_depth)
//         trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);    
//     vpMbGenericTracker tracker(trackerTypes);
//     // In Case 2 camera frames are used (color and depth) maps need to be defined
//     vpHomogeneousMatrix depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
//     std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
//     std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
//     std::map<std::string, std::string> mapOfInitFiles;
//     std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
//     std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
//     std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;    
//     // Set tracker configurations depending of which frames and features are used
//     if ((use_edges || use_klt) && use_depth) {
//         tracker.loadConfigFile(config_color, config_depth);
//         tracker.loadModel(model_color, model_depth);
//         std::cout << "Sensor internal depth_M_color: \n" << depth_M_color << std::endl;
//         mapOfCameraTransformations["Camera2"] = depth_M_color;
//         tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
//         mapOfImages["Camera1"] = &I_gray;
//         mapOfImages["Camera2"] = &I_depth;
//         mapOfInitFiles["Camera1"] = init_file;
//         tracker.setCameraParameters(cam_color, cam_depth);//     }
//     else if (use_edges || use_klt) {
//         tracker.loadConfigFile(config_color);
//         tracker.loadModel(model_color);
//         tracker.setCameraParameters(cam_color);
//     }
//     else if (use_depth) {
//         tracker.loadConfigFile(config_depth);
//         tracker.loadModel(model_depth);
//         tracker.setCameraParameters(cam_depth);
//     }
//     tracker.setDisplayFeatures(true);
//     tracker.setOgreVisibilityTest(use_ogre);
//     tracker.setScanLineVisibilityTest(use_scanline);
//     tracker.setProjectionErrorComputation(true);
//     tracker.setProjectionErrorDisplay(display_projection_error);    
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  DETECTOR, EXTRACTOR, MATCHER settings
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     #if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)) || \
//         (VISP_HAVE_OPENCV_VERSION >= 0x030411 && CV_MAJOR_VERSION < 4) || (VISP_HAVE_OPENCV_VERSION >= 0x040400)
//         std::string detectorName = "SIFT";
//         std::string extractorName = "SIFT";
//         std::string matcherName = "BruteForce";
//     #else
//         std::string detectorName = "FAST";
//         std::string extractorName = "ORB";
//         std::string matcherName = "BruteForce-Hamming";
//     #endif
//     vpKeyPoint keypoint;
//     if (learn || auto_init) 
//     {
//         keypoint.setDetector(detectorName);
//         keypoint.setExtractor(extractorName);
//         keypoint.setMatcher(matcherName);
//         // Set ORB Level Parameter
//         #if !(defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
//             #if (VISP_HAVE_OPENCV_VERSION < 0x030000)
//                 keypoint.setDetectorParameter("ORB", "nLevels", 1);
//             #else
//                 cv::Ptr<cv::ORB> orb_detector = keypoint.getDetector("ORB").dynamicCast<cv::ORB>();
//                 if (orb_detector) {
//                     orb_detector->setNLevels(1);
//                 }
//             #endif
//         #endif
//     }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------    
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  TRACKER Initialisierung (auto init oder USER CLICKS)
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     if (auto_init) // 2.option: file data-learned.bin aleready exists, you already learned the object, same initial pose
//     {
//         if (!vpIoTools::checkFilename(learning_data)) {
//             std::cout << "Cannot enable auto detection. Learning file \"" << learning_data << "\" doesn't exist" << std::endl;
//             //return EXIT_FAILURE;
//             as->setPreempted();
//         }
//         keypoint.loadLearningData(learning_data, true);
//     } else //1.option: Init tracker with clicking the 4 points in the first frame
//     {
//         if ((use_edges || use_klt) && use_depth)
//             tracker.initClick(mapOfImages, mapOfInitFiles, true);
//         else if (use_edges || use_klt)
//             tracker.initClick(I_gray, init_file, true);
//         else if (use_depth)
//             tracker.initClick(I_depth, init_file, true);
//         if (learn)  // 1.b: save initialisation in the file data-learned,bin, so that auto_init can be performed next time
//             vpIoTools::makeDirectory(vpIoTools::getParent(learning_data));
//     }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------    
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  Declare some needed variables
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     bool quit = false;
//     bool learn_position = false;
//     bool run_auto_init = false;
//     if (auto_init) { run_auto_init = true;}    
//     std::vector<vpColVector> pointcloud;
//     std::vector<double> times_vec;
//     double loop_t = 0;
//     int learn_id = 1;
//     vpHomogeneousMatrix cMo;
//     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     //  MAIN ALGORITHM - LOOP
//     // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     try {
//         while (!quit) {
//             double t = vpTime::measureTimeMs();
//             bool tracking_failed = false;
//             // --------------------------------------------------------------------------------------------------------------------------
//             //  AQUIRE IMAGES and update tracker input data
//             // --------------------------------------------------------------------------------------------------------------------------
//             realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, &pointcloud, NULL, NULL);
//             if (use_edges || use_klt || run_auto_init) {
//                 vpImageConvert::convert(I_color, I_gray);
//                 vpDisplay::display(I_gray);
//             }
//             if (use_depth) {
//                 vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
//                 vpDisplay::display(I_depth);
//             }
//             if ((use_edges || use_klt) && use_depth) {
//                 mapOfImages["Camera1"] = &I_gray;
//                 mapOfPointclouds["Camera2"] = &pointcloud;
//                 mapOfWidths["Camera2"] = width;
//                 mapOfHeights["Camera2"] = height;
//             } else if (use_edges || use_klt) {
//                 mapOfImages["Camera"] = &I_gray;
//             } else if (use_depth) {
//                 mapOfPointclouds["Camera"] = &pointcloud;
//                 mapOfWidths["Camera"] = width;
//                 mapOfHeights["Camera"] = height;
//             }
//             // Run auto initialization from learned data
//             if (run_auto_init) {
//                 if (keypoint.matchPoint(I_gray, cam_color, cMo)) // Check if auto initialisation was good via keypoint matching
//                 {
//                     std::cout << "Auto init succeed" << std::endl;
//                     if ((use_edges || use_klt) && use_depth) {
//                         mapOfCameraPoses["Camera1"] = cMo;
//                         mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
//                         tracker.initFromPose(mapOfImages, mapOfCameraPoses);
//                     } else if (use_edges || use_klt) {
//                         tracker.initFromPose(I_gray, cMo);
//                     } else if (use_depth) {
//                         tracker.initFromPose(I_depth, depth_M_color*cMo);
//                     }
//                 } else {
//                     std::cout << "Auto init did NOT succeed" << std::endl;
//                     if (use_edges || use_klt) {
//                         vpDisplay::flush(I_gray);
//                     }
//                     if (use_depth) {
//                         vpDisplay::flush(I_depth);
//                     }
//                     continue;
//                 }
//             }
//             // --------------------------------------------------------------------------------------------------------------------------
//             // Run the tracker
//             // --------------------------------------------------------------------------------------------------------------------------
//             try {
//                 if (run_auto_init) {
//                     // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
//                     tracker.setDisplayFeatures(false);
//                     run_auto_init = false;
//                 }
//                 if ((use_edges || use_klt) && use_depth) {
//                     tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
//                 } else if (use_edges || use_klt) {
//                     tracker.track(I_gray);
//                 } else if (use_depth) {
//                     tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
//                 }
//             } catch (const vpException &e) {
//                 std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
//                 tracking_failed = true;
//                 if (auto_init) {
//                     std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
//                     run_auto_init = true;
//                 }
//             }            
//             // --------------------------------------------------------------------------------------------------------------------------
//             // CHECK TRACKING ERRORS
//             // --------------------------------------------------------------------------------------------------------------------------
//             double proj_error = 0;
//             if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
//                 proj_error = tracker.getProjectionError();
//             } else {
//                 proj_error = tracker.computeCurrentProjectionError(I_gray, cMo, cam_color);
//             }
//             if (auto_init && proj_error > proj_error_threshold) {
//                 std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
//                 run_auto_init = true;
//                 tracking_failed = true;
//             }
//             // --------------------------------------------------------------------------------------------------------------------------
//             // Get and Send Transformation
//             // --------------------------------------------------------------------------------------------------------------------------
//             if (broadcast_transform){
//                 // Get object pose
//                 cMo = tracker.getPose();
//                 // std::cout << "cMo " << cMo << "\n";
//                 // std::cout << "depth_M_color -> " << depth_M_color << "\n";
//                 // 2.1 Converting the rotation matrix to Euler angle
//                 // ZYX order, that is, roll around the x axis, then around the y axis pitch, and finally around the z axis yaw, 0 for the X axis, 1 for the Y axis, 2 for the Z axis
//                 Eigen::Matrix3d rotation_matrix;
//                 vpRotationMatrix R;
//                 cMo.extract(R);
//                 for (unsigned int ii = 0; ii < R.getRows(); ii++) {
//                     for (unsigned int jj = 0; jj < R.getCols(); jj++) {
//                         rotation_matrix(ii, jj) = R[ii][jj];
//                     }
//                 }
//                 Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); 
//                 // std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << std::endl;
//                 cMo.extract(q);
//                 cMo.extract(translation);
//                 broadcast_transformation(parent_frame, child_frame, q, translation);
//             }
//             // --------------------------------------------------------------------------------------------------------------------------
//             // --------------------------------------------------------------------------------------------------------------------------
//             // DISPLAY TRACKING RESULTS
//             // --------------------------------------------------------------------------------------------------------------------------
//             if (!tracking_failed) {
//                 tracker.setDisplayFeatures(true);
//                 if ((use_edges || use_klt) && use_depth) {
//                     tracker.display(I_gray, I_depth, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::red, 3);
//                     vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
//                     vpDisplay::displayFrame(I_depth, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);
//                 } else if (use_edges || use_klt) {
//                     tracker.display(I_gray, cMo, cam_color, vpColor::red, 3);
//                     vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
//                 } else if (use_depth) {
//                     tracker.display(I_depth, cMo, cam_depth, vpColor::red, 3);
//                     vpDisplay::displayFrame(I_depth, cMo, cam_depth, 0.05, vpColor::none, 3);
//                 }
//                 std::stringstream ss;
//                 ss << "Nb features: " << tracker.getError().size();
//                 vpDisplay::displayText(I_gray, I_gray.getHeight() - 50, 20, ss.str(), vpColor::red);       
//                 ss << "Features: edges " << tracker.getNbFeaturesEdge()
//                     << ", klt " << tracker.getNbFeaturesKlt()
//                     << ", depth " << tracker.getNbFeaturesDepthDense();
//                 vpDisplay::displayText(I_gray, I_gray.getHeight() - 30, 20, ss.str(), vpColor::red);
//             }
//             std::stringstream ss;
//             ss << "Loop time: " << loop_t << " ms";
//             // --------------------------------------------------------------------------------------------------------------------------
//             // DISPLAY AND GET USER INPUT -> quit, learn, autoinit
//             // --------------------------------------------------------------------------------------------------------------------------
//             vpMouseButton::vpMouseButtonType button;
//             if (use_edges || use_klt) {
//                 vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
//                 if (learn)
//                     vpDisplay::displayText(I_gray, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
//                 else if (auto_init)
//                     vpDisplay::displayText(I_gray, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
//                 else
//                     vpDisplay::displayText(I_gray, 35, 20, "Right click: quit", vpColor::red);
//                 vpDisplay::flush(I_gray);
//                 if (vpDisplay::getClick(I_gray, button, false)) {
//                     if (button == vpMouseButton::button3) {
//                         quit = true;
//                     } else if (button == vpMouseButton::button1 && learn) {
//                         learn_position = true;
//                     } else if (button == vpMouseButton::button1 && auto_init && !learn) {
//                         run_auto_init = true;
//                     }
//                 }
//             }
//             if (use_depth) {
//                 vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);
//                 vpDisplay::displayText(I_depth, 40, 20, "Click to quit", vpColor::red);
//                 vpDisplay::flush(I_depth);
//                 if (vpDisplay::getClick(I_depth, false)) {
//                     quit = true;
//                 }
//             }            
//             loop_t = vpTime::measureTimeMs() - t;
//             times_vec.push_back(loop_t);
//         }
//     } catch (const vpException &e) {
//         std::cout << "Catch an exception: " << e.what() << std::endl;
//     }
//     // ----------------------------------------------------------------------------------------------------------------------------------------------------------
//     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//     if (!times_vec.empty()) {
//         std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
//                 << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;
//     }
//     as->setSucceeded();
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_server");
    ROS_INFO("Detection node has been started");

    // Server server(nh, "detect_object", boost::bind(&execute, _1, &server), false);
    // server.start();

    CamDetectionAction detection("detection");

    ros::spin();
    return 0;
}

// #elif defined(VISP_HAVE_REALSENSE2)
//     int main() {
//         std::cout << "Install OpenCV 3rd party, configure and build ViSP again to use this example" << std::endl;
//         return 0;
//     }
// #else
//     int main() {
//         std::cout << "Install librealsense2 3rd party, configure and build ViSP again to use this example" << std::endl;
//         return 0;
//     }
// #endif 