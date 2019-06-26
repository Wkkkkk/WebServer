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

#ifndef AUTOPLANNING_ACTION_H
#define AUTOPLANNING_ACTION_H

#include <string>
#include <memory>

#include "messages/common.h"
#define STR(str) #str

#define DEFINE_DESCRIPTOR(Type) \
std::string Type::GetDescriptor() { return STR(Type); } \
std::string Type::Descriptor = STR(Type);

// TODO: fix this awkward solution
#define DEFINE_ACTION \
public: \
virtual std::string GetDescriptor(); \
static std::string Descriptor;

class Action
{
    DEFINE_ACTION
};

class ConnectAction : public Action
{
    DEFINE_ACTION
};

class PlanningAction : public Action
{
    DEFINE_ACTION
public:
    using PointPair = typename std::pair<MapPose, MapPose>;

    explicit PlanningAction(PointPair index_pair);

    PointPair point_pair_;
};

class GoAction : public Action
{
    DEFINE_ACTION
};

class QueryAction : public Action
{
    DEFINE_ACTION
};

class ResetAction : public Action
{
    DEFINE_ACTION
};

class EndAction : public Action
{
    DEFINE_ACTION
};

std::shared_ptr<Action> convertString2Action(const std::string& s);

#endif //AUTOPLANNING_ACTION_H
