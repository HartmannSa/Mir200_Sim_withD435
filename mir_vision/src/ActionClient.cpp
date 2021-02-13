#include <mir_vision/CamDetectionAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<mir_vision::CamDetectionAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detection_client");
  Client client("detect_object", true); // true -> don't need ros::spin()
  client.waitForServer();

  mir_vision::CamDetectionGoal goal;
  goal.object_name = "Objekt";
  goal.max_time = 200;
  client.sendGoal(goal);

  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! Object found");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}