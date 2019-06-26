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

#include <ros/ros.h>
#include "RosNode.h"
#include "WebsocketServer.h"

int main(int argc, char** argv)
{
    std::cout << "initial thread: " << std::this_thread::get_id() << std::endl;
    ros::init(argc, argv, "server");

    // init && run
    RosNode ros_node;

    // io thread
    std::thread iothread([&ros_node]() {
        auto const address = boost::asio::ip::make_address("0.0.0.0");
        auto const port = 8888;

        // The io_context is required for all I/O
        boost::asio::io_context ioc;

        // Create and launch a listening port
        ros_node.listener_ = std::make_shared<listener>(ros_node, ioc, tcp::endpoint{address, port});
        ros_node.listener_->run();

        // Run the I/O service on the other thread
        ioc.run();
    });

    // main thread
    ros_node.run();

    return 0;
}