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
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>


boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
boost::mutex mutex;
ros::Publisher pub, box_pub, box_arr_pub;
jsk_pcl_ros::BoundingBoxArray::ConstPtr box_msg;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::mutex::scoped_lock(mutex);
  // control_name is "sec nsec index"
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN) {
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
    pub.publish(index);
    box_pub.publish(box_msg->boxes[index.data]);
    jsk_pcl_ros::BoundingBoxArray array_msg;
    array_msg.header = box_msg->header;
    array_msg.boxes.push_back(box_msg->boxes[index.data]);
    box_arr_pub.publish(array_msg);
  }
}

void boxCallback(const jsk_pcl_ros::BoundingBoxArray::ConstPtr& msg)
{
  boost::mutex::scoped_lock(mutex);
  box_msg = msg;
  server->clear();

  // create cube markers
  for (size_t i = 0; i < msg->boxes.size(); i++) {
    jsk_pcl_ros::BoundingBox box = msg->boxes[i];
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = box.header.frame_id;
    int_marker.pose = box.pose;
    {
      std::stringstream ss;
      ss << "menu_box" << "_" << i;
      int_marker.name = ss.str();
    }
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    
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
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name);
  }
  server->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bounding_box_interactive_marker");
  ros::NodeHandle n, pnh("~");
  server.reset(new interactive_markers::InteractiveMarkerServer("bounding_box_menu_interactive_marker", "", false));
  pub = pnh.advertise<jsk_pcl_ros::Int32Stamped>("selected_index", 1);
  box_pub = pnh.advertise<jsk_pcl_ros::BoundingBox>("selected_box", 1);
  box_arr_pub = pnh.advertise<jsk_pcl_ros::BoundingBoxArray>("selected_box_array", 1);
  ros::Subscriber sub = pnh.subscribe("bounding_box_array", 1, boxCallback);
  menu_handler.insert("First Entry", &processFeedback);
  menu_handler.insert("Secound Entry", &processFeedback);
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu");
  menu_handler.insert(sub_menu_handle, "First Entry", &processFeedback);
  menu_handler.insert(sub_menu_handle, "Secound Entry",&processFeedback);
  

  ros::spin();
  server.reset();
  return 0;
}
