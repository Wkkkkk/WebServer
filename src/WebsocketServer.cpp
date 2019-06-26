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
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>
#include <memory>

#include "WebsocketServer.h"
#include "Dispatcher.h"

// Report a failure
void fail(boost::system::error_code ec, char const* what)
{
    std::cerr << what << ": " << ec.message() << "\n";
}

session::session(const std::shared_ptr<ActionDispatcher>& dispatcher, tcp::socket socket) :
    ws_(std::move(socket)),
    strand_(ws_.get_executor()),
    dispatcher_(dispatcher) { }

void session::run()
{
    // Accept the websocket handshake
    ws_.async_accept(
            boost::asio::bind_executor(
                    strand_,
                    std::bind(
                            &session::on_accept,
                            shared_from_this(),
                            std::placeholders::_1)));
}

void session::on_accept(boost::system::error_code ec)
{
    if(ec)
    {
        return fail(ec, "session::accept");
    }
    // Read a message
    do_read();
}

void session::do_read()
{
    // Read a message into our buffer
    ws_.async_read(
            buffer_,
            boost::asio::bind_executor(
                    strand_,
                    std::bind(
                            &session::on_read,
                            shared_from_this(),
                            std::placeholders::_1,
                            std::placeholders::_2)));
}

void session::on_read(boost::system::error_code ec, std::size_t bytes_transferred)
{
    boost::ignore_unused(bytes_transferred);

    // This indicates that the session was closed
    if(ec == websocket::error::closed)
        return;

    if(ec)
        fail(ec, "read");

    if(!buffer_.size()) return;

    // unpack message
    ws_.text(ws_.got_text());
    std::stringstream ss;
    ss << boost::beast::buffers(buffer_.data());
    std::string message = ss.str();
    // std::cout << "session::on_read in thread: " << std::this_thread::get_id() << std::endl;

    // Clear the buffer
    buffer_.consume(buffer_.size());

    // dispatch message
    auto dispatcher = dispatcher_.lock();
    if(dispatcher) {
        dispatcher->onMessage(shared_from_this(), message);
    }

    // Do another read
    do_read();
}

void session::write_message(const std::string& message) {
    // future/then thread
    ws_.async_write(
            boost::asio::buffer(message),
            boost::asio::bind_executor(
                    strand_,
                    std::bind(
                            &session::on_write,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2)));
}

void session::on_write(boost::system::error_code ec, std::size_t bytes_transferred)
{
    // io thread
    boost::ignore_unused(bytes_transferred);

    if(ec)
        return fail(ec, "write");

    // Clear the buffer
    // buffer_.consume(buffer_.size());

    // Do another read
    // do_read();
}

listener::listener(RosNode& rosnode, boost::asio::io_context& ioc, tcp::endpoint endpoint) :
    rosnode_(rosnode),
    acceptor_(ioc),
    socket_(ioc)
{
    boost::system::error_code ec;

    // Open the acceptor
    acceptor_.open(endpoint.protocol(), ec);
    if(ec)
    {
        fail(ec, "open");
        return;
    }

    // Enable address reuse
    acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));

    // Bind to the server address
    acceptor_.bind(endpoint, ec);
    if(ec)
    {
        fail(ec, "bind");
        return;
    }

    // Start listening for connections
    acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
    if(ec)
    {
        fail(ec, "listen");
        return;
    }
}

void listener::run()
{
    init();

    if(! acceptor_.is_open())
        return;
    do_accept();
}

void listener::do_accept()
{
    acceptor_.async_accept(
            socket_,
            std::bind(
                    &listener::on_accept,
                    shared_from_this(),
                    std::placeholders::_1));
}

void listener::on_accept(boost::system::error_code ec)
{
    if(ec)
    {
        fail(ec, "listener::accept");
    }
    else
    {
        std::cout << "accept a connection in thread: " << std::this_thread::get_id() << std::endl;

        // Create the session and run it
        std::make_shared<session>(dispatcher_, std::move(socket_))->run();
    }

    // Accept another connection
    do_accept();
}

void listener::update() {
    // main thread
    FunctionWrapper task;
    if (work_queue.try_pop(task)) {
        task();
    }

    if (dispatcher_) dispatcher_->update();
}

void listener::init() {
    dispatcher_.reset(new ActionDispatcher(shared_from_this()));
}
