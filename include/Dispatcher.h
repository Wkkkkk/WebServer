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

#ifndef AUTOPLANNING_DISPATCHER_H
#define AUTOPLANNING_DISPATCHER_H

#include <map>
#include <memory>
#include <functional>
#include <type_traits>
#include <iostream>
#include <cassert>

#include "Action.h"

class session;
class listener;

typedef std::weak_ptr<session>  SocketConnectPtr;
typedef std::weak_ptr<listener> ListenerPtr;
typedef std::shared_ptr<listener> ListeneSharedrPtr;
typedef std::shared_ptr<Action> MessagePtr;
typedef std::shared_ptr<ConnectAction> ConnectPtr;
typedef std::shared_ptr<PlanningAction> PlanningPtr;
typedef std::shared_ptr<QueryAction> QueryPtr;
typedef std::shared_ptr<ResetAction> ResetPtr;
typedef std::shared_ptr<GoAction> GoPtr;
typedef std::shared_ptr<EndAction> EndPtr;

class Callback
{
public:
    virtual ~Callback() = default;
    virtual void onMessage(const SocketConnectPtr&, const MessagePtr& message) const = 0;
};

template <typename T>
class CallbackT : public Callback
{
    static_assert(std::is_base_of<Action, T>::value,
                  "T must be derived from Action.");
public:
    typedef std::function<void (const SocketConnectPtr&, const std::shared_ptr<T>& message )> MessageTCallback;

    CallbackT(const MessageTCallback& callback)
            : callback_(callback)
    {
    }

    void onMessage(const SocketConnectPtr& conn, const MessagePtr& message) const override
    {
        std::shared_ptr<T> concrete = ::std::static_pointer_cast<T>(message);
        assert(concrete != NULL);
        callback_(conn, concrete);
    }

private:
    MessageTCallback callback_;
};

class ActionDispatcher
{
public:
    typedef std::function<void (const SocketConnectPtr&, const MessagePtr& message)> MessageCallback;
    explicit ActionDispatcher(const ListeneSharedrPtr& listener);

    void onMessage(const SocketConnectPtr& conn, const std::string& message);

    void writeToConnect(const SocketConnectPtr& conn, const std::string& message);
    void writeToCurrentConnect(const std::string& message);

    template<typename T>
    void registerMessageCallback(const typename CallbackT<T>::MessageTCallback& callback)
    {
        std::shared_ptr<CallbackT<T> > pd(new CallbackT<T>(callback));
        callbacks_[T::Descriptor] = pd;
    }

    void update();
    void onAction(const SocketConnectPtr&, const MessagePtr& message);
    void onConnect(const SocketConnectPtr& conn, const ConnectPtr& message);
    void onPlanning(const SocketConnectPtr& conn, const PlanningPtr& message);
    void onQuery(const SocketConnectPtr& conn, const QueryPtr& message);
    void onReset(const SocketConnectPtr& conn, const ResetPtr& message);
    void onGo(const SocketConnectPtr& conn, const GoPtr& message);
    void onEnd(const SocketConnectPtr& conn, const EndPtr& message);

private:
    typedef std::map<std::string, std::shared_ptr<Callback> > CallbackMap;

    SocketConnectPtr cur_connect_;
    ListenerPtr cur_listener_;
    CallbackMap callbacks_;
    MessageCallback defaultCallback_;
    bool enable_status_update_ = false;
};


#endif //AUTOPLANNING_DISPATCHER_H
