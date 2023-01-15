/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file cyphal_subscribers.h
 * @author d.ponomarev
 * @date Dec 28, 2021
 */

#ifndef LIBCYPHAL_CYPHAL_SUBSCRIBERS_HPP_
#define LIBCYPHAL_CYPHAL_SUBSCRIBERS_HPP_

#include "canard.h"
#include "uavcan/node/GetInfo_1_0.h"
#include "uavcan/node/ExecuteCommand_1_0.h"

class CyphalNode;

class CyphalSubscriber {
public:
    CyphalSubscriber(CyphalNode* driver_, CanardPortID port_id_) : driver(driver_), port_id(port_id_) {}
    virtual void callback(const CanardRxTransfer& transfer) = 0;
    bool isEnabled();
    CanardRxSubscription subscription;
    CyphalNode* driver;
    CanardPortID port_id;
};

#endif  // LIBCYPHAL_CYPHAL_SUBSCRIBERS_HPP_
