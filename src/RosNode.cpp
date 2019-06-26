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
#include <map>
#include <regex>
#include <functional>

#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <automsgs/lane.h>
#include <automsgs/SimCar.h>
#include <automsgs/CloudClusterArray.h>
#include <automsgs/ConfigWaypointFollower.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/date_time.hpp>
#include <boost/variant.hpp>

#include "RosNode.h"
#include "WebsocketServer.h"

boost::property_tree::ptree map_to_json(const std::map<std::string, boost::property_tree::ptree>& handler) {
    boost::property_tree::ptree ptree;
    for(const auto& pair : handler) {
        ptree.put_child(pair.first, pair.second);
    }

    return ptree;
}

boost::property_tree::ptree map_to_json(const std::map<std::string, std::function<std::string()>>& handler) {
    boost::property_tree::ptree ptree;
    for(const auto& pair : handler) {
        auto func = pair.second;
        if(func) {
            ptree.put(pair.first, func());
        }
    }

    return ptree;
}

std::string json_to_string(const boost::property_tree::ptree& ptree) {
    std::stringstream ss;
    boost::property_tree::write_json(ss, ptree, false);

    std::string s = ss.str();
    return s;
}

boost::property_tree::ptree get_null_json() {
    boost::property_tree::ptree ptree;
    return ptree;
}

struct Status {
    std::map<std::string, std::string> handler {
            {"time", ""},
            {"company", "Zhihui"},
            {"version", "1.0"},
            {"type", "default-status"},
            {"data", ""}
    };

    Status (const std::string& type, const boost::property_tree::ptree& data) {
        boost::posix_time::ptime timeLocal = boost::posix_time::second_clock::local_time();
        handler["time"] = boost::posix_time::to_simple_string(timeLocal);
        handler["type"] = type;
        data_tree_ = data;
    }

    std::string to_string() {
        boost::property_tree::ptree ptree;
        for(const auto& pair : handler) {
            ptree.put(pair.first, pair.second);
        }

        ptree.put_child("data", data_tree_);
        std::string s = json_to_string(ptree);

        return s;
    }

    boost::property_tree::ptree data_tree_;
};

RosNode::RosNode()
{
    std::cout << "RosNode" << std::endl;
    // global path
    way_planner_sub = nodeHandle.subscribe("/lane_waypoints_array", 10, &RosNode::wayPlannerCallback, this);
    // local path
    final_waypoints_sub = nodeHandle.subscribe("/final_waypoints", 10, &RosNode::finalWaypointsCallback, this);
    // pose
    current_pos_sub = nodeHandle.subscribe("current_pose", 10, &RosNode::callbackGetCurrentPos, this);
    // speed a
    twist_sub = nodeHandle.subscribe("twist_raw", 10, &RosNode::callbackGetTwistRaw, this);
    // steering_angle speed a
    ctrl_sub = nodeHandle.subscribe("ctrl_cmd", 10, &RosNode::callbackGetCtrlCmd, this);
    // state
    state_sub = nodeHandle.subscribe("behavior_state", 1, &RosNode::callbackGetState, this);
    // target point
    target_point_sub = nodeHandle.subscribe("next_target_point", 1, &RosNode::callbackGetTargetPoint, this);
    target_point_sub2 = nodeHandle.subscribe("follow_pose", 1, &RosNode::callbackGetTargetPoint2, this);
    // bounding box
    bboxes_sub = nodeHandle.subscribe("/bounding_boxes", 10, &RosNode::callbackGetBBoxes, this);

    start_pos_pub = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/GlobalStartPose", 1, true);
    goal_pos_pub = nodeHandle.advertise<geometry_msgs::PoseStamped>("/GlobalGoalPose", 1, true);
    reset_pub = nodeHandle.advertise<std_msgs::Bool>("/StatusReset", 1, true);
}

RosNode::~RosNode() = default;

void RosNode::run() {
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        // update
        update();

        loop_rate.sleep();
    }
}

void RosNode::update() {
    if(listener_) listener_->update();
}

void RosNode::setCurrentRoute(MapPose p1, MapPose p2) {
    start_point_ = p1;
    target_point_ = p2;
    ROS_INFO("Start: %f, %f, %f ; %f, %f, %f, %f", p1.position.x, p1.position.y, p1.position.z,
            p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
    ROS_INFO("Target: %f, %f, %f ; %f, %f, %f, %f", p2.position.x, p2.position.y, p2.position.z,
            p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);
}

void RosNode::go() {
    std::cout << "You may go now" << std::endl;

    auto transform = [](const MapPose& point) -> geometry_msgs::PoseWithCovarianceStamped {
        geometry_msgs::PoseWithCovarianceStamped p;
        p.pose.pose.position.x = point.position.x;
        p.pose.pose.position.y = point.position.y;
        p.pose.pose.position.z = point.position.z;
        p.pose.pose.orientation.x = point.orientation.x;
        p.pose.pose.orientation.y = point.orientation.y;
        p.pose.pose.orientation.z = point.orientation.z;
        p.pose.pose.orientation.w = point.orientation.w;

        return p;
    };

    auto start_point = transform(start_point_);
    auto target_point = transform(target_point_);
    start_pos_pub.publish(start_point);
    goal_pos_pub.publish(target_point);
}

void RosNode::reset() {
    std::cout << "reset all" << std::endl;

    std_msgs::Bool reset;
    reset.data = 1;
    reset_pub.publish(reset);
}

std::string RosNode::getCurrentHardwareStatus() {
    Status status("hardware-status", get_current_hardware_status());
    return status.to_string();
}

std::string RosNode::getCurrentReadyStatus() {
    Status status("ready-status", get_null_json());
    return status.to_string();
}

std::string RosNode::getCurrentRunningStatus() {
    Status status("running-status", get_running_status());
    return status.to_string();
}

std::string RosNode::getCurrentFinishStatus() {
    Status status("finish-status", get_null_json());
    return status.to_string();
}

boost::property_tree::ptree RosNode::get_obstacles() {
    boost::property_tree::ptree obstacle_array;

    for(const auto& box : obstacles_.boxes) {

        boost::property_tree::ptree obstacle;
        obstacle.put("type", "PEDESTRAIN");

        boost::property_tree::ptree point_array_json;
        auto point_list = { box.pt1, box.pt2, box.pt3, box.pt4, box.pt5, box.pt6, box.pt7, box.pt8 };
        for(const auto& point : point_list) {
            boost::property_tree::ptree point_json;
            boost::property_tree::ptree x, y, z;

            x.put("", point.x);
            y.put("", point.y);
            z.put("", point.z);

            point_json.push_back(std::make_pair("", x));
            point_json.push_back(std::make_pair("", y));
            point_json.push_back(std::make_pair("", y));

            point_array_json.push_back(std::make_pair("", point_json));
        }

        obstacle.push_back(std::make_pair("points", point_array_json));
        obstacle_array.push_back(std::make_pair("", obstacle));
    }

    return obstacle_array;
}

boost::property_tree::ptree  RosNode::get_current_hardware_status() {
    boost::property_tree::ptree root;
    return root;
}

boost::property_tree::ptree  RosNode::get_local() {
    boost::property_tree::ptree root;

    auto local_path = local_path_;
    root.put("id", local_path.lane_id);
    root.put("name", "TRAJECTORY");

    boost::property_tree::ptree point_array_json;
    auto points = local_path_.points;
    for(int i = 0; i < points.size(); i += 5)
    {
        boost::property_tree::ptree point_json;
        boost::property_tree::ptree x, y, z;
        auto point = points[i];

        x.put("", point.position.x);
        y.put("", point.position.y);
        z.put("", point.position.z);

        point_json.push_back(std::make_pair("", x));
        point_json.push_back(std::make_pair("", y));
        point_json.push_back(std::make_pair("", z));

        point_array_json.push_back(std::make_pair("", point_json));
    }
    root.put_child("points", point_array_json);

    return root;
}

boost::property_tree::ptree RosNode::get_car() {
    boost::property_tree::ptree root;

    boost::property_tree::ptree item1;
    auto position = current_pose_.position;
    item1.put("x", position.x);
    item1.put("y", position.y);
    item1.put("z", position.z);
    root.push_back(std::make_pair("position",item1));

    boost::property_tree::ptree item2;
    auto orientation = current_pose_.orientation;
    item2.put("x", orientation.x);
    item2.put("y", orientation.y);
    item2.put("z", orientation.z);
    item2.put("w", orientation.w);
    root.push_back(std::make_pair("rotation",item2));

    return root;
}

boost::property_tree::ptree RosNode::get_status() {
    boost::property_tree::ptree root;
    auto direction = current_direction_;
    root.put("speed", direction.linear_velocity);
    root.put("accelerator", direction.linear_acceleration);
    root.put("wheel", direction.steering_angle);
    root.put("brake", direction.angular_velocity);
    root.put("status", current_status_str_);

    return root;

}

boost::property_tree::ptree RosNode::get_running_status() {

    std::map<std::string, boost::property_tree::ptree> handler {
            {"obstacles", get_obstacles()},
            {"local", get_local()},
            {"car", get_car()},
            {"status", get_status()},
    };

    boost::property_tree::ptree ptree = map_to_json(handler);
    return ptree;
}

void RosNode::wayPlannerCallback(const automsgs::LaneArrayConstPtr &msg) {
    MapLanes lanes;
    for (const auto& lane : msg->lanes) {
        MapLane mapLane;
        mapLane.lane_id = lane.lane_id;
        for (const auto& waypoint : lane.waypoints) {
            MapPose mapPose;
            mapPose.orientation = MapVec4(waypoint.pose.pose.orientation.x, waypoint.pose.pose.orientation.y,
                                          waypoint.pose.pose.orientation.z, waypoint.pose.pose.orientation.w);
            mapPose.position = MapVec3(waypoint.pose.pose.position.x, waypoint.pose.pose.position.y,
                                       waypoint.pose.pose.position.z);
            mapLane.points.push_back(mapPose);
        }
        lanes.lanes.push_back(mapLane);
    }

    global_paths_ = lanes;
}

void RosNode::finalWaypointsCallback(const automsgs::laneConstPtr &msg) {
    MapLane finalWaypoint;
    finalWaypoint.lane_id = msg->lane_id;
    for (const auto &waypoint : msg->waypoints) {
        MapPose vPose;
        vPose.position.x = waypoint.pose.pose.position.x;
        vPose.position.y = waypoint.pose.pose.position.y;
        vPose.position.z = waypoint.pose.pose.position.z;
        vPose.orientation.x = waypoint.twist.twist.linear.x; // velocity
        vPose.orientation.y = waypoint.twist.twist.linear.y;
        vPose.orientation.z = waypoint.twist.twist.linear.z;
        finalWaypoint.points.push_back(vPose);
    }

    local_path_ = finalWaypoint;
}

void RosNode::callbackGetCurrentPos(const geometry_msgs::PoseStampedConstPtr &msg) {
    MapPose current_pose;
    current_pose.position = MapPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    current_pose.orientation = MapOrientation(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                              msg->pose.orientation.w);

    current_pose_ = current_pose;
}

void RosNode::callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr &msg) {
    MapStatus status;
    status.linear_velocity = msg->twist.linear.x;
    status.angular_velocity = msg->twist.angular.z;

    current_direction_ = status;
}

void RosNode::callbackGetCtrlCmd(const automsgs::ControlCommandStampedConstPtr &msg) {
    MapStatus status;
    status.steering_angle = msg->cmd.steering_angle;
    status.linear_velocity = msg->cmd.linear_velocity;
    status.linear_acceleration = msg->cmd.linear_acceleration;

    current_direction_ = status;
}

void RosNode::callbackGetState(const visualization_msgs::Marker &msg) {
    current_status_str_ = msg.text;
}

void RosNode::callbackGetTargetPoint2(const visualization_msgs::Marker &msg) {
    MapPosition pos;
    pos.x = msg.pose.position.x;
    pos.y = msg.pose.position.y;
    pos.z = msg.pose.position.z;

    follow_point_ = pos;
}

void RosNode::callbackGetTargetPoint(const geometry_msgs::Point &msg) {
    MapPosition pos;
    pos.x = msg.x;
    pos.y = msg.y;
    pos.z = msg.z;

    follow_point_ = pos;
}

void RosNode::callbackGetBBoxes(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &msg) {

    MapBoxes bboxes;
    for (const auto& box : msg->boxes){
        double l2 = box.dimensions.x / 2.0;
        double w2 = box.dimensions.y / 2.0;
        double h2 = box.dimensions.z / 2.0;
        MapPosition pt1(-w2, -l2, box.pose.position.z);
        MapPosition pt2(w2, -l2, box.pose.position.z);
        MapPosition pt3(w2, l2, box.pose.position.z);
        MapPosition pt4(-w2, l2, box.pose.position.z);

        tf::Quaternion orgin_quaternion(box.pose.orientation.x, box.pose.orientation.y,
                                        box.pose.orientation.z, box.pose.orientation.w);
        double yaw = -tf::getYaw(orgin_quaternion);
        double m[3][3];
        double c = cos(yaw);
        double s = sin(yaw);
        m[0][0] = c; m[0][1] = -s; m[0][2] =  0;
        m[1][0] = s; m[1][1] =  c; m[1][2] =  0;
        m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;

        MapPosition npt1 = pt1;
        npt1.x = m[0][0]*pt1.x + m[0][1]*pt1.y + m[0][2]*1;
        npt1.y = m[1][0]*pt1.x + m[1][1]*pt1.y + m[1][2]*1;
        MapPosition npt2 = pt2;
        npt2.x = m[0][0]*pt2.x + m[0][1]*pt2.y + m[0][2]*1;
        npt2.y = m[1][0]*pt2.x + m[1][1]*pt2.y + m[1][2]*1;
        MapPosition npt3 = pt3;
        npt3.x = m[0][0]*pt3.x + m[0][1]*pt3.y + m[0][2]*1;
        npt3.y = m[1][0]*pt3.x + m[1][1]*pt3.y + m[1][2]*1;
        MapPosition npt4 = pt4;
        npt4.x = m[0][0]*pt4.x + m[0][1]*pt4.y + m[0][2]*1;
        npt4.y = m[1][0]*pt4.x + m[1][1]*pt4.y + m[1][2]*1;

        MapBox mbox;
        mbox.pt1 = npt1;
        mbox.pt1.x += box.pose.position.x; mbox.pt1.y += box.pose.position.y; mbox.pt1.z -= h2 - 0.05;
        mbox.pt2 = npt2;
        mbox.pt2.x += box.pose.position.x; mbox.pt2.y += box.pose.position.y; mbox.pt2.z -= h2 - 0.05;
        mbox.pt3 = npt3;
        mbox.pt3.x += box.pose.position.x; mbox.pt3.y += box.pose.position.y; mbox.pt3.z -= h2 - 0.05;
        mbox.pt4 = npt4;
        mbox.pt4.x += box.pose.position.x; mbox.pt4.y += box.pose.position.y; mbox.pt4.z -= h2 - 0.05;

        mbox.pt5 = npt1;
        mbox.pt5.x += box.pose.position.x; mbox.pt5.y += box.pose.position.y; mbox.pt5.z += h2;
        mbox.pt6 = npt2;
        mbox.pt6.x += box.pose.position.x; mbox.pt6.y += box.pose.position.y; mbox.pt6.z += h2;
        mbox.pt7 = npt3;
        mbox.pt7.x += box.pose.position.x; mbox.pt7.y += box.pose.position.y; mbox.pt7.z += h2;
        mbox.pt8 = npt4;
        mbox.pt8.x += box.pose.position.x; mbox.pt8.y += box.pose.position.y; mbox.pt8.z += h2;

        bboxes.boxes.push_back(mbox);
    }

    std::swap(obstacles_, bboxes);
}