/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file cyphal_subscribers.cpp
 * @author d.ponomarev
 * @date Aug 12, 2022
 */

#include "cyphal/cyphal_subscribers.hpp"
#include "cyphal/cyphal_node.hpp"

bool CyphalSubscriber::isEnabled() {
    constexpr uint16_t MAX_PORT_ID = 8191;
    return (port_id == 0 || port_id > MAX_PORT_ID) ? false : true;
}
