/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <jsk_pcl_ros/Int32Stamped.h>
#include <jsk_pcl_ros/BoundingBoxArray.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/bind.hpp>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler lv0_menu_handler;
interactive_markers::MenuHandler lv1_menu_handler;
interactive_markers::MenuHandler lv2_menu_handler;
boost::mutex mutex;
ros::Publisher lv0_pub, lv0_box_pub, lv0_box_arr_pub;
ros::Publisher lv1_pub, lv1_box_pub, lv1_box_arr_pub;
ros::Publisher menu_action_pub;
ros::Publisher manipulate_action_pub;
jsk_pcl_ros::BoundingBoxArray::ConstPtr box_msg[2];
bool update_box_ = true;
bool wait_for_interact_ = false;
ros::Time last_int_t_; //last interaction time



void boxMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const int level)
{
  boost::mutex::scoped_lock(mutex);
  // control_name is "sec nsec index"
  
  last_int_t_ = ros::Time::now();
  
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
    // if (level == 0){
    //   update_box_ = false;
    //  }
    ROS_INFO("update_box_:%d", update_box_);
    std::string control_name = feedback->control_name;
    ROS_INFO_STREAM("control_name: " << control_name);
    std::list<std::string> splitted_string;
    boost::split(splitted_string, control_name, boost::is_space());
    jsk_pcl_ros::Int32Stamped index;
    index.header.stamp.sec = boost::lexical_cast<int>(splitted_string.front());
    splitted_string.pop_front();
    index.header.stamp.nsec = boost::lexical_cast<int>(splitted_string.front());
    splitted_string.pop_front();
    index.data = boost::lexical_cast<int>(splitted_string.front());
    if (level == 0){
      lv0_pub.publish(index);
      lv0_box_pub.publish(box_msg[0]->boxes[index.data]);
    //makeMenuMarker(box_msg->boxes[index.data]);
      jsk_pcl_ros::BoundingBoxArray array_msg;
      array_msg.header = box_msg[0]->header;
      array_msg.boxes.push_back(box_msg[0]->boxes[index.data]);
      lv0_box_arr_pub.publish(array_msg);
    }
    if (level == 1){
      lv1_pub.publish(index);
      lv1_box_pub.publish(box_msg[1]->boxes[index.data]);
    //makeMenuMarker(box_msg->boxes[index.data]);
      jsk_pcl_ros::BoundingBoxArray array_msg;
      array_msg.header = box_msg[1]->header;
      array_msg.boxes.push_back(box_msg[1]->boxes[index.data]);
      lv1_box_arr_pub.publish(array_msg);
    }
    if (level == 2){
    }
  }
}

void makeInterativeBox(const int level)
{
  server->clear();
  // create cube markers
  for (size_t i = 0; i < box_msg[level]->boxes.size(); i++) {
    jsk_pcl_ros::BoundingBox box = box_msg[level]->boxes[i];
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = box.header.frame_id;
    int_marker.pose = box.pose;
    {
      std::stringstream ss;
      ss <<"level"<< boost::format("%02d") % level <<"_" << "box" << "_" << i;
      int_marker.name = ss.str();
    }
    
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    {
      std::stringstream ss;
      // encode several informations into control name
      ss << box.header.stamp.sec << " " << box.header.stamp.nsec << " " << i;
      control.name = ss.str();
    }
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = box.dimensions.x + 0.01;
    marker.scale.y = box.dimensions.y + 0.01;
    marker.scale.z = box.dimensions.z + 0.01;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.0;
    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);
    server->insert(int_marker);
    server->setCallback(int_marker.name, boost::bind(&boxMarkerFeedback, _1, level));
    
    if (level == 0){
      lv0_menu_handler.apply( *server, int_marker.name);
    }else if (level == 1){
      lv1_menu_handler.apply( *server, int_marker.name);
    }
  }
  server->applyChanges();
}

void arrowMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const int level)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN){
    geometry_msgs::PoseStamped res;
    std::vector<std::string> splitted_string;
    std::string control_name = feedback -> control_name;
    boost::split(splitted_string, control_name, boost::is_space());
    res.header.stamp.sec = boost::lexical_cast<int>(splitted_string[0]);
    res.header.stamp.nsec = boost::lexical_cast<int>(splitted_string[1]);
    res.header.frame_id = feedback -> header.frame_id;
    res.pose = feedback -> pose;
    manipulate_action_pub.publish(res);
    wait_for_interact_ = false;
    update_box_ = true;
  }
}

visualization_msgs::InteractiveMarker makeArrowIntMarker(tf::Transform t, const jsk_pcl_ros::BoundingBox box, int n)
{
  visualization_msgs::InteractiveMarker int_marker;
  geometry_msgs::Pose arrow_pose;
  arrow_pose.position.x = t.getOrigin()[0];
  arrow_pose.position.y = t.getOrigin()[1];
  arrow_pose.position.z = t.getOrigin()[2];
  arrow_pose.orientation.x = t.getRotation()[0];
  arrow_pose.orientation.y = t.getRotation()[1];
  arrow_pose.orientation.z = t.getRotation()[2];
  arrow_pose.orientation.w = t.getRotation()[3];
  int_marker.header.frame_id = box.header.frame_id;
  int_marker.pose = arrow_pose;
  {
  std::stringstream ss;
  // encode several informations into control name
  ss << "arrow_" << n;
  int_marker.name = ss.str();
  }
  visualization_msgs::InteractiveMarkerControl control;
  {
  std::stringstream ss;
  // encode several informations into control name
  ss << box.header.stamp.sec << " " << box.header.stamp.nsec;
  control.name = ss.str();
  }
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.2;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = (n == 0 || n == 1)? 1.0 : 0.0;
  marker.color.g = (n == 4 || n == 5)? 1.0 : 0.0;
  marker.color.b = (n == 2 || n == 3)? 1.0 : 0.0;
  marker.color.a = 1.0;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);
  return int_marker;
}


void makeInteractiveArrow(const int level, const jsk_pcl_ros::BoundingBox box)
{
  boost::mutex::scoped_lock(mutex);
  server->clear();
  // ROS_INFO("BOX_INDEX:%d", box_idx);
  // jsk_pcl_ros::BoundingBox box = box_msg[level]->boxes[box_idx];
  geometry_msgs::Pose pose = box.pose;
  tf::Transform box_trans;
  tf::Vector3 box_pos = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
  tf::Quaternion box_orient = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Transform y90deg;
  tf::Transform z90deg;
  tf::Transform xshift;
  box_trans.setOrigin(box_pos);
  box_trans.setRotation(box_orient);
  y90deg.setOrigin(tf::Vector3(0,0,0));
  y90deg.setRotation(tf::createQuaternionFromRPY(0,M_PI/2,0));
  z90deg.setOrigin(tf::Vector3(0,0,0));
  z90deg.setRotation(tf::createQuaternionFromRPY(0,0,M_PI/2));
  xshift.setOrigin(tf::Vector3(box.dimensions.x / 2, 0, 0));
  xshift.setRotation(tf::createQuaternionFromRPY(0,0,0));
  
  visualization_msgs::InteractiveMarker int_marker;
  //x-axis
  int_marker= makeArrowIntMarker(box_trans * xshift, box , 0);
  server -> insert(int_marker);
  server -> setCallback(int_marker.name, boost::bind(&arrowMarkerFeedback, _1, level));
  //negative-x-axis
  int_marker = makeArrowIntMarker(box_trans * y90deg * y90deg * xshift, box, 1);
  server -> insert(int_marker);
  server -> setCallback(int_marker.name, boost::bind(&arrowMarkerFeedback, _1, level));
  xshift.setOrigin(tf::Vector3(box.dimensions.z / 2, 0, 0));
  //negative-z-axis
  int_marker = makeArrowIntMarker(box_trans * y90deg * xshift, box, 2);
  server -> insert(int_marker);
  server -> setCallback(int_marker.name, boost::bind(&arrowMarkerFeedback, _1, level));
  //z-axis
  int_marker = makeArrowIntMarker(box_trans * y90deg * y90deg *  y90deg * xshift, box, 3);
  server -> insert(int_marker);
  server -> setCallback(int_marker.name, boost::bind(&arrowMarkerFeedback, _1, level));
  xshift.setOrigin(tf::Vector3(box.dimensions.y / 2, 0, 0));
  //y-axis
  int_marker = makeArrowIntMarker(box_trans * z90deg * xshift, box, 4);
  server -> insert(int_marker);
  server -> setCallback(int_marker.name, boost::bind(&arrowMarkerFeedback, _1, level));
  //negative-y-axis
  int_marker = makeArrowIntMarker(box_trans * z90deg * z90deg * z90deg* xshift, box, 5);
  server -> insert(int_marker);
  server -> setCallback(int_marker.name, boost::bind(&arrowMarkerFeedback, _1, level));
  server -> applyChanges();
}
void makeManipulateActionMenu(int level,int box_idx)
{
  boost::mutex::scoped_lock(mutex);
  server->clear();

  jsk_pcl_ros::BoundingBox box = box_msg[level]->boxes[box_idx];
  visualization_msgs::InteractiveMarkerControl control;
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = box.header.frame_id;
  int_marker.pose = box.pose;
    {
      std::stringstream ss;
      ss <<"level"<< boost::format("%02d") % level <<"_" << "box" << "_" << box_idx;
      int_marker.name = ss.str();
    }

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  {
  std::stringstream ss;
  // encode several informations into control name
  ss << box.header.stamp.sec << " " << box.header.stamp.nsec << " " << box_idx;
  control.name = ss.str();
  }
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = box.dimensions.x + 0.05;
  marker.scale.y = box.dimensions.y + 0.05;
  marker.scale.z = box.dimensions.z + 0.05;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.0;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);
  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&boxMarkerFeedback, _1, 1));
  lv2_menu_handler.apply( *server, int_marker.name);
  server->applyChanges();
}





void menuMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const int level, const int order)
{
  boost::mutex::scoped_lock(mutex);
  ROS_INFO("MENU_FEEDBACK:%d", feedback->event_type);
  ROS_INFO("MENU_Level:%d", level);
  std::vector<std::string> splitted_string;
  std::string control_name = feedback -> control_name;
  boost::split(splitted_string, control_name, boost::is_space());
  jsk_pcl_ros::Int32Stamped req;
  req.header.stamp.sec = boost::lexical_cast<int>(splitted_string[0]);
  req.header.stamp.nsec = boost::lexical_cast<int>(splitted_string[1]);
  int box_idx= boost::lexical_cast<int>(splitted_string[2]);
  if (level == 0){
    switch (order){
    case 0: //grasp
      req.data = 0;
      menu_action_pub.publish(req);
      break;
    case 1:
      req.data = 1; //release object
      menu_action_pub.publish(req);
      break;
    case 2: //investigate
      req.data = 2;
      menu_action_pub.publish(req);
      makeInterativeBox(1);
      break;
    }
  }
  if (level == 1){
    switch (order){
    case 0:
      req.data = 3; //grasp left
      wait_for_interact_ = true;
      makeManipulateActionMenu(level, box_idx);
      menu_action_pub.publish(req);
      break;
    case 1:
      req.data = 4; //grasp right
      wait_for_interact_ = true;
      makeManipulateActionMenu(level, box_idx);
      menu_action_pub.publish(req);
      break;
    }
  }
  if (level == 2){
    switch (order){
    case 0: 
      req.data = 5; //manipulate
      wait_for_interact_ = true;
      makeInteractiveArrow(level-1, box_msg[level-1]->boxes[box_idx]);
      menu_action_pub.publish(req);
      break;
    case 1:
      req.data = 6; //abort=release-object
      wait_for_interact_ = false;
      last_int_t_ = ros::Time::now() + ros::Duration(-5.0);
      menu_action_pub.publish(req);
      break;
    }
  }
}


void boxCallback00(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
{
  boost::mutex::scoped_lock(mutex);

  ROS_INFO("time:%lf", ros::Time::now().toSec() - last_int_t_.toSec());
  box_msg[0] = msg;

  if ((!wait_for_interact_) && (ros::Time::now().toSec() - last_int_t_.toSec()) >= 5.0){
    makeInterativeBox(0);
  }
}

void boxCallback01(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
{
  boost::mutex::scoped_lock(mutex);
  box_msg[1] = msg;
}

void drawArrowCallback(const jsk_pcl_ros::BoundingBox box)
{
  makeInteractiveArrow(1, box);
  last_int_t_ = ros::Time::now();
}

void initMenu ()
{
  lv0_menu_handler.insert("Grasp object", boost::bind(&menuMarkerFeedback, _1, 0, 0));
  // interactive_markers::MenuHandler::EntryHandle lv0_arm_menu_handle = lv0_menu_handler.insert("PlaceObject");
  // lv0_menu_handler.insert(lv0_arm_menu_handle, "LeftArm", boost::bind(&menuMarkerFeedback, _1, 0, 1));
  // lv0_menu_handler.insert(lv0_arm_menu_handle, "RightArm", boost::bind(&menuMarkerFeedback, _1, 0, 2));
  lv0_menu_handler.insert("Release object", boost::bind(&menuMarkerFeedback, _1, 0, 1));
  lv0_menu_handler.insert("Investigate", boost::bind(&menuMarkerFeedback, _1, 0, 2));
  
  interactive_markers::MenuHandler::EntryHandle lv1_arm_menu_handle = lv1_menu_handler.insert("Grasp handle");
  lv1_menu_handler.insert(lv1_arm_menu_handle, "LeftArm", boost::bind(&menuMarkerFeedback, _1, 1, 0));
  lv1_menu_handler.insert(lv1_arm_menu_handle, "RightArm", boost::bind(&menuMarkerFeedback, _1, 1, 1));
  //lv1_menu_handler.insert("MANIPULATE", boost::bind(&menuMarkerFeedback, _1, 1, 2));
  
  lv2_menu_handler.insert("Manipulate", boost::bind(&menuMarkerFeedback, _1, 2, 0));
  lv2_menu_handler.insert("Abort", boost::bind(&menuMarkerFeedback, _1, 2, 1));
  
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "bounding_box_interactive_marker");
  ros::NodeHandle n, pnh("~");
  server.reset(new interactive_markers::InteractiveMarkerServer("bounding_box_menu_interactive_marker", "", false));
  lv0_pub = pnh.advertise<jsk_pcl_ros::Int32Stamped>("level00_selected_index", 1);
  lv1_pub = pnh.advertise<jsk_pcl_ros::Int32Stamped>("level01_selected_index", 1);
  lv0_box_pub = pnh.advertise<jsk_pcl_ros::BoundingBox>("level00_selected_box", 1);
  lv1_box_pub = pnh.advertise<jsk_pcl_ros::BoundingBox>("level01_selected_box", 1);
  lv0_box_arr_pub = pnh.advertise<jsk_pcl_ros::BoundingBoxArray>("level00_selected_box_array", 1);
  lv1_box_arr_pub = pnh.advertise<jsk_pcl_ros::BoundingBoxArray>("level01_selected_box_array", 1);
  menu_action_pub = pnh.advertise<jsk_pcl_ros::Int32Stamped>("menu_action_request", 1);
  manipulate_action_pub = pnh.advertise<geometry_msgs::PoseStamped>("manipulate_pose", 1);
  ros::Subscriber lv0_sub = pnh.subscribe("level00_bounding_box_array", 1, boxCallback00);
  ros::Subscriber lv1_sub = pnh.subscribe("level01_bounding_box_array", 1, boxCallback01);
  ros::Subscriber draw_arrow_sub = pnh.subscribe("draw_manipulate_arrow", 1, drawArrowCallback);
  initMenu();
  ros::spin();
  server.reset();
  return 0;
}
