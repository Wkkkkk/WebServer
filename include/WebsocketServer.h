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

#ifndef AUTOPLANNING_WEBSOCKETSERVER_H
#define AUTOPLANNING_WEBSOCKETSERVER_H

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>

#define BOOST_THREAD_PROVIDES_FUTURE
#define BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION
#include <boost/thread/future.hpp>

#include "FunctionWrapper.h"
#include "ThreadsafeQueue.h"

using tcp = boost::asio::ip::tcp;               // from <boost/asio/ip/tcp.hpp>
namespace websocket = boost::beast::websocket;  // from <boost/beast/websocket.hpp>

class ActionDispatcher;
class RosNode;

typedef std::weak_ptr<ActionDispatcher>  DispatcherPtr;
typedef std::shared_ptr<ActionDispatcher> DispatcherSharedPtr;

//------------------------------------------------------------------------------
class session : public std::enable_shared_from_this<session>
{
    websocket::stream<tcp::socket> ws_;
    boost::asio::strand<boost::asio::io_context::executor_type> strand_;
    boost::beast::multi_buffer buffer_;

    DispatcherPtr dispatcher_;
public:
    // Take ownership of the socket
    explicit session(const DispatcherSharedPtr& dispatcher, tcp::socket socket);

    // Start the asynchronous operation
    void run();

    void on_accept(boost::system::error_code ec);

    void do_read();

    void write_message(const std::string& message);

    void on_read(boost::system::error_code ec, std::size_t bytes_transferred);

    void on_write(boost::system::error_code ec, std::size_t bytes_transferred);
};

//------------------------------------------------------------------------------

// Accepts incoming connections and launches the sessions
class listener : public std::enable_shared_from_this<listener>
{
    friend class ActionDispatcher;
    tcp::acceptor acceptor_;
    tcp::socket socket_;
    DispatcherSharedPtr dispatcher_;
    RosNode& rosnode_;
    ThreadsafeQueue<FunctionWrapper> work_queue;

public:
    listener(RosNode& rosnode, boost::asio::io_context& ioc, tcp::endpoint endpoint);

    // init dispatcher
    void init();

    // Start accepting incoming connections
    void run();

    // Accept query or command
    template <typename FunctionType>
    auto submit(FunctionType &&f)
    -> boost::future<typename std::result_of<FunctionType()>::type>
    {
        // main thread
        using result_type = typename std::result_of<FunctionType()>::type;

        boost::packaged_task<result_type> task(std::forward<FunctionType>(f));
        boost::future<result_type> res(task.get_future());
        work_queue.push(std::move(task));

        return res;
    }

    void update();

    void do_accept();

    void on_accept(boost::system::error_code ec);
};


#endif //AUTOPLANNING_WEBSOCKETSERVER_H
