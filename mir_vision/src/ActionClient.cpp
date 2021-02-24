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
  const int STATE_SEARCHING = 0;
  const int STATE_FINISH = 1;
  const int STATE_FOUND_MATCH = 2;
  const int STATE_REFINE = 3;

  const int MAX_TRIAL = 1;

  DetectionClient client_detect;
  mir_vision::CamDetectionGoal detection_goal;
  mir_vision::CamDetectionFeedback feedbackPrevious_;

  MoveBaseClient client_move;
  move_base_msgs::MoveBaseGoal move_goal;
  geometry_msgs::PoseArray posearray_;
  int move_goal_number;
  int trial;
  
  bool reached_end_of_searchpath;

  bool moveRobot_;


public:
  
  // ------------------------------------------------------ //
  //  Konstruktor und Destruktor                            //
  // ------------------------------------------------------ //   
  CamDetectionClient() : client_detect("detection", true), 
                        client_move("move_base", true)
  {
    ROS_INFO("Waiting for detection action server to start.");
    client_detect.waitForServer();
    ROS_INFO("Detection action server started, sending goal.");
  }

  ~CamDetectionClient() {}
  
  // ------------------------------------------------------ //
  //  DetectObject ("Main Funktion")                        //
  // ------------------------------------------------------ //
  void detectObject(std::string objekt_name, std::string learning_data, std::string path_poses, bool move_robot)
  {  
    moveRobot_ = move_robot;   
    move_goal_number = 0;
    trial = 0;
    reached_end_of_searchpath = false;
    loadPoses(path_poses);

    detection_goal.object_name = objekt_name;
    detection_goal.learning_data = learning_data;    
    client_detect.sendGoal(detection_goal, 
                    boost::bind(&CamDetectionClient::doneCb, this, _1, _2),
                    boost::bind(&CamDetectionClient::activeCb, this),
                    boost::bind(&CamDetectionClient::feedbackCb, this, _1));     
  }

  // ------------------------------------------------------ //
  //  Load search poses from config file                           //
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
      posearray_.header.stamp = ros::Time::now(); 
      posearray_.header.frame_id = "map"; 
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
          posearray_.poses.push_back(p);
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
    // Posearry already initialised by loadPoses()
    move_goal.target_pose.pose = posearray_.poses[goal_number];
  }  

  // ------------------------------------------------------ //
  //  doneCallback                                          //
  // ------------------------------------------------------ //
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const mir_vision::CamDetectionResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("rotation stdev: %f; %f; %f", result->rotation_stdev.x, result->rotation_stdev.y, result->rotation_stdev.z );
    ROS_INFO("translation stdev: %f; %f; %f", result->translation_stdev.x, result->translation_stdev.y, result->translation_stdev.z );
    ros::shutdown();
  }

  // ------------------------------------------------------ //
  //  activeCallback                                        //
  // ------------------------------------------------------ //
  void activeCb()
  {
    ROS_INFO("Goal just went active");

    if (moveRobot_){
      while(!client_move.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      setMoveGoal(move_goal_number);
      ROS_INFO("Sending first nav-goal with number %i", move_goal_number);   
      client_move.sendGoal(move_goal);
    }    
  } 
  
  // ------------------------------------------------------ //
  //  feedbackCallback                                      //
  // ------------------------------------------------------ //
  void feedbackCb(const mir_vision::CamDetectionFeedbackConstPtr& feedback)
  {
    //  if (feedback->state != feedbackPrevious_.state) {ROS_INFO("Got Feedback with state %i", feedback->state);}
    ROS_INFO("Got Feedback with state %i", feedback->state);
    // ROS_INFO("Object position:    [%.2f; %.2f; %.2f]", feedback->estimated_pose.pose.position.x, feedback->estimated_pose.pose.position.y, feedback->estimated_pose.pose.position.z );
    // ROS_INFO("Object orientation: [%.2f; %.2f; %.2f; %.2f]", feedback->estimated_pose.pose.orientation.x, feedback->estimated_pose.pose.orientation.y, feedback->estimated_pose.pose.orientation.z, feedback->estimated_pose.pose.orientation.w );
    
    if (moveRobot_)
    { 
      actionlib::SimpleClientGoalState moveState = client_move.getState();          
      switch (feedback->state){
        case 0: //STATE_SEARCHING           
          if (moveState == actionlib::SimpleClientGoalState::SUCCEEDED ||
              moveState == actionlib::SimpleClientGoalState::ABORTED ||
              moveState == actionlib::SimpleClientGoalState::REJECTED ||
              moveState == actionlib::SimpleClientGoalState::RECALLED ||
              moveState == actionlib::SimpleClientGoalState::PREEMPTED) 
          {           
            ROS_INFO("Navigation state: %s", moveState.toString().c_str());
            
            if (moveState == actionlib::SimpleClientGoalState::SUCCEEDED){ 
              move_goal_number++;} 
            else if (moveState == actionlib::SimpleClientGoalState::ABORTED ||
                    moveState == actionlib::SimpleClientGoalState::REJECTED) {
              trial++;
              move_goal_number++;
            }
            
            // Send search pose, if the end of search poses is not reached already 
            // e.g. size returns 4, than is the last goal to send 3 (because we start with goal 0)
            if (move_goal_number < posearray_.poses.size()){    
              setMoveGoal(move_goal_number);
              ROS_INFO("Sending nav-goal with number %i", move_goal_number);    
              client_move.sendGoal(move_goal);
            } else {              
              client_detect.cancelGoal();
              ROS_INFO("End of search path reached. Nav-goal number %i = %zu (number of search poses).", move_goal_number, posearray_.poses.size() );
              ROS_INFO("Cancel detection Goal %s.", detection_goal.object_name.c_str()); 
              move_goal_number = 0; // Continue search at Startposition  
            } 
          }           
          break;
        case 1: //(STATE_FINISH):
          break;
        case 2: //(STATE_FOUND_MATCH):
          ROS_INFO("Navigation state: %s", moveState.toString().c_str());
          // If Navigation is still active,cancel it because an object is detected
          if (moveState == actionlib::SimpleClientGoalState::ACTIVE) {
            client_move.cancelGoal();
            // if (move_goal_number != 0) 
            //   move_goal_number--;
            ROS_INFO("Cancel nav-goal %i", move_goal_number);
          } 
          break;
        case 3: //(STATE_REFINE):
          break;
        default:
          ROS_ERROR("The Feedback state %i is not defined!", feedback->state );
          break;
      }
    } 
    feedbackPrevious_.estimated_pose = feedback->estimated_pose;            
    feedbackPrevious_.state = feedback->state;
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
  std::string object_name;
  std::string learning_data;
  bool move_robot;
  nh.param<std::string>("object_name", object_name, "Teabox");
  nh.param<std::string>("learning_data", learning_data, "Teabox0_learning_data.bin");
  nh.param<std::string>("path_searchposes", path, "/home/rosmatch/visp-ws/src/Mir200_Sim_withD435/mir_vision/config/searchPoses.config");
  nh.param<bool>("move_robot", move_robot, true);
  
  CamDetectionClient cam_detection_client;
  cam_detection_client.detectObject(object_name, learning_data, path, move_robot);
  ros::spin();
  return 0;
}