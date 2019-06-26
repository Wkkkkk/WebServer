/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 5/5/19.
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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/variant.hpp>
#include <boost/date_time.hpp>

#include "Action.h"

DEFINE_DESCRIPTOR(Action)
DEFINE_DESCRIPTOR(ConnectAction)
DEFINE_DESCRIPTOR(PlanningAction)
DEFINE_DESCRIPTOR(GoAction)
DEFINE_DESCRIPTOR(QueryAction)
DEFINE_DESCRIPTOR(ResetAction)
DEFINE_DESCRIPTOR(EndAction)

PlanningAction::PlanningAction(PlanningAction::PointPair point_pair) :
    point_pair_(std::move(point_pair)) {}


std::shared_ptr<Action> convertString2Action(const std::string& s) {
    std::stringstream ss(s);
    std::shared_ptr<Action> action(new Action);

    boost::property_tree::ptree ptree;
    try{
        boost::property_tree::read_json(ss, ptree);

        auto type = ptree.get_child_optional("type");
        if(type) {
            auto value = type.value().get_value<std::string>();
            std::cout << "---------------------" << std::endl;
            std::cout << "get action: " << value << std::endl;

            if(value == "connect") {
                action.reset(new ConnectAction);
            } else if (value == "planning") {
                MapPose p1, p2;
                {
                    auto x =  ptree.get<double>("data.p1.position.x");
                    auto y =  ptree.get<double>("data.p1.position.y");
                    auto z =  ptree.get<double>("data.p1.position.z");
                    p1.position = MapVec3(x, y, z);

                    auto o_x =  ptree.get<double>("data.p1.rotation.x");
                    auto o_y =  ptree.get<double>("data.p1.rotation.y");
                    auto o_z =  ptree.get<double>("data.p1.rotation.z");
                    auto o_w =  ptree.get<double>("data.p1.rotation.w");
                    p1.orientation = MapVec4(o_x, o_y, o_z, o_w);
                }
                {
                    auto x =  ptree.get<double>("data.p2.position.x");
                    auto y =  ptree.get<double>("data.p2.position.y");
                    auto z =  ptree.get<double>("data.p2.position.z");
                    p2.position = MapVec3(x, y, z);

                    auto o_x =  ptree.get<double>("data.p2.rotation.x");
                    auto o_y =  ptree.get<double>("data.p2.rotation.y");
                    auto o_z =  ptree.get<double>("data.p2.rotation.z");
                    auto o_w =  ptree.get<double>("data.p2.rotation.w");
                    p2.orientation = MapVec4(o_x, o_y, o_z, o_w);
                }

                action.reset(new PlanningAction({p1, p2}));
            } else if (value == "go") {
                action.reset(new GoAction);
            } else if (value == "query") {
                action.reset(new QueryAction);
            } else if (value == "reset") {
                action.reset(new ResetAction);
            } else if (value == "end") {
                action.reset(new EndAction);
            }
        }
    }
    catch(boost::property_tree::ptree_error & e) {
        std::cout << "invalid input: " << s << std::endl;
        return action;
    }

    return action;
}