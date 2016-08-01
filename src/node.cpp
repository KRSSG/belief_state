#include "ros/ros.h"
#include "../../ssl_common/include/ssl_common/config.h"
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <queue>

const int BALL_AT_CORNER_THRESH                        = 20; 
const int HALF_FIELD_MAXX                              = 3000; 
const int HALF_FIELD_MAXY                              = 2000;
const float MAX_DRIBBLE_R                              = 3;
const int DBOX_WIDTH                                   = 600;
const int DBOX_HEIGHT                                  = 600;
using namespace std;

bool is_team_yellow;
ros::Subscriber vision_sub;
ros::Publisher pub;
queue<pair<geometry_msgs::Pose2D, ros::Time> > velQ;

const int Q_SIZE = 3;
void Callback(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg) 
{
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

    if(vmsg->balls[0].x <= 0 )
        msg.ball_in_our_half = true;
    else
        msg.ball_in_our_half = false;

    if(fabs(vmsg->balls[0].x) > (HALF_FIELD_MAXX - BALL_AT_CORNER_THRESH) && fabs(vmsg->balls[0].y) > (HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH))
        msg.ball_at_corners = true;
    else
        msg.ball_at_corners = false;

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

  float distance_from_ball = 999999,dist,temp=10;
  msg.ball_in_our_possession = false;

  for (int i = 0; i < homePos.size(); ++i)
  {
    int bot_id = homePos[i].robot_id;
    msg.homeDetected[bot_id] = 1;
    msg.homePos[bot_id].x = homePos[i].x;
    msg.homePos[bot_id].y = homePos[i].y;

    dist = sqrt(pow((homePos[i].x - vmsg->balls[0].x),2) - pow((homePos[i].y - vmsg->balls[0].y) , 2));
    if(dist < distance_from_ball){
      distance_from_ball = dist;
      msg.our_bot_closest_to_ball = i;

      if(distance_from_ball < MAX_DRIBBLE_R){
        msg.ball_in_our_possession = true;
      }
    }

    msg.homePos[bot_id].theta = homePos[i].orientation;

    if(homePos[i].x < (-HALF_FIELD_MAXX + DBOX_WIDTH) && fabs(homePos[i].y) < DBOX_HEIGHT){
        
        if(temp = 10)
            temp = i;
        else if(fabs(homePos[temp].orientation) > 1.54 && fabs(homePos[i].orientation) < 1.54)
            temp = i;
    }
  }
  msg.our_goalie = temp;
  temp = 10;
  distance_from_ball = 999999;
  for (int i = 0; i < awayPos.size(); ++i)
  {
    int bot_id = awayPos[i].robot_id;
    msg.awayDetected[bot_id] = 1;
    msg.awayPos[bot_id].x = awayPos[i].x;
    msg.awayPos[bot_id].y = awayPos[i].y;

    dist = sqrt(pow((awayPos[i].x - vmsg->balls[0].x),2) - pow((awayPos[i].y - vmsg->balls[0].y) , 2));
    if(dist < distance_from_ball){
      distance_from_ball = dist;
      msg.opp_bot_closest_to_ball = i;
    }

    msg.awayPos[bot_id].theta = awayPos[i].orientation;

    if(homePos[i].x > (HALF_FIELD_MAXX - DBOX_WIDTH) && fabs(homePos[i].y) < DBOX_HEIGHT){
        
        if(temp = 10)
            temp = i;
        else if(fabs(homePos[temp].orientation) > 1.54 && fabs(homePos[i].orientation) < 1.54)
            temp = i;
    }

  }
  msg.opp_goalie = temp;

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