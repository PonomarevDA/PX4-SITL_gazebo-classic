/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file cyphal_transport.hpp
 * @author d.ponomarev
 * @date Jul 07, 2022
 */

#ifndef CYPHAL_TRANSPORT_HPP_
#define CYPHAL_TRANSPORT_HPP_

#include "canard.h"
#include "socketcan.h"

class CyphalTransportCan {
public:
    CyphalTransportCan() {};
    bool init(uint32_t can_speed, uint8_t can_driver_idx);
    bool receive(CanardFrame* can_frame);
    bool transmit(const uint64_t current_time_us, const CanardTxQueueItem* transfer);
private:
    uint8_t _out_payload[256];
    uint8_t _can_driver_idx;
    SocketCANFD _instance = 0;
};

#endif  // CYPHAL_TRANSPORT_HPP_