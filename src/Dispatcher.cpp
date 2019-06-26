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

#include "Dispatcher.h"
#include "WebsocketServer.h"
#include "RosNode.h"

namespace ph = std::placeholders;

void onDefaultMessageType(const SocketConnectPtr&, const MessagePtr& message)
{
    std::cout << "unknown message!"<< std::endl;
}

ActionDispatcher::ActionDispatcher(const ListeneSharedrPtr& listener) :
    cur_listener_(listener),
    defaultCallback_(onDefaultMessageType)
{
    // Register callbacks
    this->registerMessageCallback<Action>(std::bind(&ActionDispatcher::onAction, this, ph::_1, ph::_2));
    this->registerMessageCallback<ConnectAction>(std::bind(&ActionDispatcher::onConnect, this, ph::_1, ph::_2));
    this->registerMessageCallback<PlanningAction>(std::bind(&ActionDispatcher::onPlanning, this, ph::_1, ph::_2));
    this->registerMessageCallback<QueryAction>(std::bind(&ActionDispatcher::onQuery, this, ph::_1, ph::_2));
    this->registerMessageCallback<ResetAction>(std::bind(&ActionDispatcher::onReset, this, ph::_1, ph::_2));
    this->registerMessageCallback<GoAction>(std::bind(&ActionDispatcher::onGo, this, ph::_1, ph::_2));
    this->registerMessageCallback<EndAction>(std::bind(&ActionDispatcher::onEnd, this, ph::_1, ph::_2));
}

void ActionDispatcher::writeToConnect(const SocketConnectPtr &conn, const std::string &message) {
    // future/then thread
    auto connect = conn.lock();

    if(connect) {
        connect->write_message(message);
    } else {
        std::cout << "connect disconnected." << std::endl;
    }
}

void ActionDispatcher::writeToCurrentConnect(const std::string &message) {
    writeToConnect(cur_connect_, message);
}

void ActionDispatcher::onMessage(const SocketConnectPtr &conn, const std::string &message)
{
    // io thread
    std::shared_ptr<Action> action = convertString2Action(message);
//    std::cout << "unpack: " << action->GetDescriptor() << " at: " << std::this_thread::get_id() << std::endl;

    auto connect = conn.lock();
    if(connect) {
        // mark this as current connection waiting for write
        cur_connect_ = connect;

        auto it = callbacks_.find(action->GetDescriptor());
        if (it != callbacks_.end())
        {
            it->second->onMessage(conn, action);
        }
        else
        {
            defaultCallback_(conn, action);
        }
    }
}

void ActionDispatcher::onAction(const SocketConnectPtr & con, const MessagePtr &message) {
    onDefaultMessageType(con, message);
}

void ActionDispatcher::onConnect(const SocketConnectPtr &conn, const ConnectPtr &message) {
    std::cout << "onConnect: " << message->GetDescriptor() << std::endl;
    enable_status_update_ = false;

    auto cur_listener = cur_listener_.lock();
    if(cur_listener) {

        // hardware-status query
        auto result = cur_listener->submit(std::bind(&RosNode::getCurrentHardwareStatus, &cur_listener->rosnode_));

        // response
        result.then([this, &conn](boost::future<std::string> future) {
            // future/then thread
            std::string str = future.get();
            writeToConnect(conn, str);
        });
    }
}

void ActionDispatcher::onPlanning(const SocketConnectPtr &conn, const PlanningPtr &message) {
    std::cout << "onPlanning: " << message->GetDescriptor() << std::endl;
    // valid check
    auto index_pair = message->point_pair_;
    auto p1 = index_pair.first;
    auto p2 = index_pair.second;

    auto cur_listener = cur_listener_.lock();
    if(cur_listener) {

        // ready-status query
        auto result = cur_listener->submit(std::bind(&RosNode::getCurrentReadyStatus, &cur_listener->rosnode_));

        // response
        result.then([this, &conn](boost::future<std::string> future) {
            // future/then thread
            std::string str = future.get();
            writeToConnect(conn, str);
        });

        // Action: set route
        cur_listener->submit(std::bind(&RosNode::setCurrentRoute, &cur_listener->rosnode_, p1, p2));
    }
}

void ActionDispatcher::onGo(const SocketConnectPtr &conn, const GoPtr &message) {
    std::cout << "onGo: " << message->GetDescriptor() << std::endl;

    auto cur_listener = cur_listener_.lock();
    if(cur_listener) {

        // Action: go!
        cur_listener->submit(std::bind(&RosNode::go, &cur_listener->rosnode_));
    }
}

void ActionDispatcher::onQuery(const SocketConnectPtr &conn, const QueryPtr &message) {
    std::cout << "onQuery: " << message->GetDescriptor() << std::endl;

    auto cur_listener = cur_listener_.lock();
    if(cur_listener) {
        // enable/disable status-update
        enable_status_update_ = true;
    }
}

void ActionDispatcher::onReset(const SocketConnectPtr &conn, const ResetPtr &message) {
    std::cout << "onReset: " << message->GetDescriptor() << std::endl;

    auto cur_listener = cur_listener_.lock();
    if(cur_listener) {

        // Action: reset!
        cur_listener->submit(std::bind(&RosNode::reset, &cur_listener->rosnode_));
    }
}



void ActionDispatcher::onEnd(const SocketConnectPtr &conn, const EndPtr &message) {
    std::cout << "onEnd: " << message->GetDescriptor() << std::endl;
    // disable status-update
    enable_status_update_ = false;

    auto cur_listener = cur_listener_.lock();
    if(cur_listener) {

        // finish-status query
        auto result = cur_listener->submit(std::bind(&RosNode::getCurrentFinishStatus, &cur_listener->rosnode_));

        // response
        result.then([this, &conn](boost::future<std::string> future) {
            // future/then thread
            std::string str = future.get();
            writeToConnect(conn, str);
        });
    }
}

void ActionDispatcher::update() {
    // main thread
    if(enable_status_update_) {
        auto cur_listener = cur_listener_.lock();
        if(cur_listener) {

            // running-status query
            auto result = cur_listener->submit(std::bind(&RosNode::getCurrentRunningStatus, &cur_listener->rosnode_));

            // response
            result.then([this](boost::future<std::string> future) {
                // future/then thread
                std::string str = future.get();
                writeToCurrentConnect(str);
            });
        }
    }
}