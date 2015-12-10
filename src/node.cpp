#include "ros/ros.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <queue>
using namespace std;

bool is_team_yellow;
ros::Subscriber vision_sub;
ros::Publisher pub;
queue<pair<geometry_msgs::Pose2D, ros::Time> > velQ;

const int Q_SIZE = 3; 
void Callback(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg) {
  printf("got a vmsg!\n");
  using namespace krssg_ssl_msgs;
  using geometry_msgs::Pose2D;
  using geometry_msgs::Point32;
  krssg_ssl_msgs::BeliefState msg;
  msg.frame_number = vmsg->frame_number;
  msg.t_capture = vmsg->t_capture;
  msg.t_sent = vmsg->t_sent;
  msg.isteamyellow = is_team_yellow;
  if (vmsg->balls.size() > 0) {
    assert(velQ.size() != 0);
    Pose2D oldPos = velQ.front().first;
    ros::Time oldTime = velQ.front().second;
    velQ.pop();
    ros::Time curTime = ros::Time::now();
    msg.ballDetected = true;
    msg.ballPos.x = vmsg->balls[0].x;
    msg.ballPos.y = vmsg->balls[0].y;
    msg.ballVel.x = (msg.ballPos.x - oldPos.x)/(curTime-oldTime).toSec();
    msg.ballVel.y = (msg.ballPos.y - oldPos.y)/(curTime-oldTime).toSec();
    velQ.push(make_pair(msg.ballPos, curTime));
  } else {
    msg.ballDetected = 0;
  }
  vector<SSL_DetectionRobot> homePos, awayPos;
  if (is_team_yellow) {
    homePos = vmsg->robots_yellow;
    awayPos = vmsg->robots_blue;
  } else {
    homePos = vmsg->robots_blue;
    awayPos = vmsg->robots_yellow;
  }
  // assuming 6 robots per side!
  msg.awayDetected = vector<uint8_t>(6, 0);
  msg.homeDetected = vector<uint8_t>(6, 0);
  msg.awayPos = vector<Pose2D>(6, Pose2D());
  msg.homePos = vector<Pose2D>(6, Pose2D());
  for (int i = 0; i < homePos.size(); ++i)
  {
    int bot_id = homePos[i].robot_id;
    msg.homeDetected[bot_id] = 1;
    msg.homePos[bot_id].x = homePos[i].x;
    msg.homePos[bot_id].y = homePos[i].y;
    msg.homePos[bot_id].theta = homePos[i].orientation;
  }
  for (int i = 0; i < awayPos.size(); ++i)
  {
    int bot_id = awayPos[i].robot_id;
    msg.awayDetected[bot_id] = 1;
    msg.awayPos[bot_id].x = awayPos[i].x;
    msg.awayPos[bot_id].y = awayPos[i].y;
    msg.awayPos[bot_id].theta = awayPos[i].orientation;
  }
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  // if no argument is passed, assumed our team is blue
  // else if argument 0 = our team blue, 1 = our team yellow
  ros::init(argc, argv, "beliefstate_node");
  is_team_yellow = 0;
  if (argc > 1) {
    is_team_yellow = atof(argv[1]);
  }
  ros::NodeHandle n;
  for (int i = 0; i < Q_SIZE; ++i)
  {
    velQ.push(make_pair(geometry_msgs::Pose2D(), ros::Time::now()));
  }
  vision_sub = n.subscribe("/vision", 1000, Callback);
  pub = n.advertise<krssg_ssl_msgs::BeliefState>("/belief_state", 1000);
  ros::spin();

  return 0;
}