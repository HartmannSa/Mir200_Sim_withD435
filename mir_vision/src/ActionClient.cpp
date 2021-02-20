#include <mir_vision/CamDetectionAction.h> 
#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseArray.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <fstream>

typedef actionlib::SimpleActionClient<mir_vision::CamDetectionAction> DetectionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""//
//  CLASS CamDetectionClient                                                                                         //
// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""" //
class CamDetectionClient
{
private:
  DetectionClient client_detect;
  mir_vision::CamDetectionGoal detection_goal;

  MoveBaseClient client_move;
  move_base_msgs::MoveBaseGoal move_goal;
  geometry_msgs::PoseArray posearray;
  int move_goal_number;
  bool reached_end_of_searchpath;

public:
  // ------------------------------------------------------ //
  //  Konstruktor und Destruktor                            //
  // ------------------------------------------------------ //   
  CamDetectionClient() : client_detect("detection", true), 
                        client_move("move_base", true)
  {
    ROS_INFO("Waiting for action server to start.");
    client_detect.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  ~CamDetectionClient() {}
  
  // ------------------------------------------------------ //
  //  DetectObject ("Main Funktion")                        //
  // ------------------------------------------------------ //
  void detectObject(std::string objekt_name, int max_time, std::string path_poses)
  {    
    move_goal_number = 0;
    reached_end_of_searchpath = false;
    loadPoses(path_poses);

    detection_goal.object_name = objekt_name;
    detection_goal.max_time = max_time;  
    client_detect.sendGoal(detection_goal, 
                    boost::bind(&CamDetectionClient::doneCb, this, _1, _2),
                    boost::bind(&CamDetectionClient::activeCb, this),
                    boost::bind(&CamDetectionClient::feedbackCb, this, _1));     
  }

  // ------------------------------------------------------ //
  //  Load Poses from config file                           //
  // ------------------------------------------------------ //
  void loadPoses(std::string path)
  {
    std::ifstream file;    
    float px, py, pz, qx, qy, qz, qw;
    char c;
    geometry_msgs::Pose p;

    file.open(path.c_str(), std::ifstream::in);   

    if (file.is_open())
    {
      posearray.header.stamp = ros::Time::now(); 
      posearray.header.frame_id = "map"; 
      std::string line;    
      while ( getline (file,line) )
      {
        std::istringstream iss(line, std::istringstream::in);
        
        if ( !line.length() || (line[0] == '#') )
            continue;   // skip empty lines or lines starting with # (comments)

        while ( (iss >> px >> c >> py >> c >> pz >> c >> qx >> c >> qy >> c >> qz >> c >> qw) && (c == ',') )
        {
          p.position.x = px;
          p.position.y = py;
          p.position.z = pz;
          p.orientation.x = qx;
          p.orientation.y = qy;
          p.orientation.z = qz;
          p.orientation.w = qw;
          posearray.poses.push_back(p);
        }        
      }
      file.close();
    } else ROS_INFO("Unable to open file %s", path.c_str() ); 
  }

  // ------------------------------------------------------ //
  //  Set a Goal for move_Base                              //
  // ------------------------------------------------------ //
  void setMoveGoal(int goal_number, std::string frame = "map")
  {
    move_goal.target_pose.header.stamp = ros::Time::now(); 
    move_goal.target_pose.header.frame_id = frame;
    move_goal.target_pose.pose = posearray.poses[goal_number];
  }  

  // ------------------------------------------------------ //
  //  doneCallback                                          //
  // ------------------------------------------------------ //
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const mir_vision::CamDetectionResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ros::shutdown();
  }

  // ------------------------------------------------------ //
  //  activeCallback                                        //
  // ------------------------------------------------------ //
  void activeCb()
  {
    ROS_INFO("Goal just went active");

    while(!client_move.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    setMoveGoal(move_goal_number);
    ROS_INFO("Sending goal number %i", move_goal_number);   
    client_move.sendGoal(move_goal);
  } 
  
  // ------------------------------------------------------ //
  //  feedbackCallback                                      //
  // ------------------------------------------------------ //
  void feedbackCb(const mir_vision::CamDetectionFeedbackConstPtr& feedback)
  {
    ROS_INFO("Got Feedback with state %i", feedback->state);
    ROS_INFO("Navigation is in state %s", client_move.getState().toString().c_str());

    if (client_move.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || client_move.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
      move_goal_number++;
      // Check if the last search pose of the posearray was already send
      if (move_goal_number >= posearray.poses.size()){
        move_goal_number = 0; // Continue search at Startposition
        reached_end_of_searchpath = true;
      }

      if (!reached_end_of_searchpath)
      {
        setMoveGoal(move_goal_number);
        ROS_INFO("Sending goal number %i", move_goal_number);    
        client_move.sendGoal(move_goal);
      } else {
        client_detect.cancelGoal();
        ROS_INFO("Cancel detection Goal %s", detection_goal.object_name.c_str());   
      }
    }          
  }
};



// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""//
//  MAIN                                                                                                             //
// """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""" //
int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_client");
  ros::NodeHandle nh("~");
  std::string path;
  nh.param<std::string>("path_searchposes", path, "/home/rosmatch/visp-ws/src/Mir200_Sim_withD435/mir_vision/config/searchPoses.config");
  
  CamDetectionClient cam_detection_client;
  cam_detection_client.detectObject("Teabox", 200, path);
  ros::spin();
  return 0;
}