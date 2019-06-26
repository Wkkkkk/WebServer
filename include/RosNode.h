/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 4/30/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef AUTOPLANNING_ROSNODE_H
#define AUTOPLANNING_ROSNODE_H


#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <automsgs/lane.h>
#include <automsgs/LaneArray.h>
#include <automsgs/V2V_ego_info.h>
#include <automsgs/V2V_nearby_info.h>
#include <automsgs/ControlCommandStamped.h>
#include <automsgs/LocationScan.h>
#include <automsgs/lane.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <vector_map_msgs/PointArray.h>
#include <vector_map_msgs/LaneArray.h>
#include <vector_map_msgs/NodeArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <boost/property_tree/ptree.hpp>

#include "messages/common.h"

class listener;

class RosNode {
public:
    RosNode();
    ~RosNode();

    void run();

    // query response
    std::string getCurrentHardwareStatus();
    std::string getCurrentReadyStatus();
    std::string getCurrentRunningStatus();
    std::string getCurrentFinishStatus();

    // action response
    void setCurrentRoute(MapPose p1, MapPose p2);
    void go();
    void reset();

    // web server
    std::shared_ptr<listener> listener_;

private:
    void wayPlannerCallback(const automsgs::LaneArrayConstPtr &msg);
    void finalWaypointsCallback(const automsgs::laneConstPtr &msg);
    void callbackGetCurrentPos(const geometry_msgs::PoseStampedConstPtr &msg);
    void callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackGetCtrlCmd(const automsgs::ControlCommandStampedConstPtr &msg);
    void callbackGetState(const visualization_msgs::Marker &msg);
    void callbackGetTargetPoint2(const visualization_msgs::Marker &msg);
    void callbackGetTargetPoint(const geometry_msgs::Point &msg);
    void callbackGetBBoxes(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &msg);

    void update();

    boost::property_tree::ptree get_current_hardware_status();
    boost::property_tree::ptree get_running_status();
    boost::property_tree::ptree get_obstacles();
    boost::property_tree::ptree get_local();
    boost::property_tree::ptree get_car();
    boost::property_tree::ptree get_status();

private:
    // ros
    ros::NodeHandle nodeHandle;

    // Sub
    //////////////////////////////////////////////
    // global path
    ros::Subscriber way_planner_sub;
    // local path
    ros::Subscriber final_waypoints_sub;
    // pose
    ros::Subscriber current_pos_sub;
    // steer speed accelerate
    ros::Subscriber twist_sub;
    ros::Subscriber ctrl_sub;
    // state
    ros::Subscriber state_sub;
    // target point
    ros::Subscriber target_point_sub;
    ros::Subscriber target_point_sub2;
    // bounding box
    ros::Subscriber bboxes_sub;

    //Pub
    //////////////////////////////////////////////
    ros::Publisher start_pos_pub;
    ros::Publisher goal_pos_pub;
    ros::Publisher reset_pub;

    MapLanes global_paths_;
    MapLane local_path_;
    MapPose current_pose_;
    MapStatus current_direction_;
    std::string current_status_str_;
    MapPosition follow_point_;
    MapBoxes obstacles_;

    MapPose start_point_;
    MapPose target_point_;
};


#endif //AUTOPLANNING_ROSNODE_H
