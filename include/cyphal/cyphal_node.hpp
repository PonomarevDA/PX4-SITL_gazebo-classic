/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file cyphal_node.hpp
 * @author d.ponomarev
 * @date Jul 07, 2022
 */

#ifndef CYPHAL_NODE_HPP_
#define CYPHAL_NODE_HPP_

#include "canard.h"
#include "cyphal/cyphal_transport.hpp"
#include "cyphal/cyphal_subscribers.hpp"
#include "cyphal/cyphal_publishers.hpp"
#include "socketcan.h"
#include "o1heap.h"

#define HEAP_SIZE           (1024*3)
#define TX_QUEUE_FRAME_SIZE 320  ///< we need 314 bytes for port.List

class CyphalNode {
public:
    CyphalNode(): heartbeat_pub(this), port_list_pub(this) {};
    bool init();
    void process();
    int32_t push(CanardTransferMetadata *metadata, size_t payload_size, const void *payload);
    int8_t subscribe(CyphalSubscriber* sub_info, size_t size, CanardTransferKind kind);

    static constexpr size_t MAX_SUB_NUM = 10;
    CyphalSubscriber* _sub_info[MAX_SUB_NUM];
    size_t _sub_num{0};
private:
    void spinReceivedFrame(const CanardMicrosecond rx_timestamp_usec,
                           const CanardFrame* const received_frame);
    void spinTransmit();
    void processReceivedTransfer(const uint8_t redundant_interface_index,
                                 const CanardRxTransfer& transfer);
    int8_t subscribeApplication();
    CanardInstance canard_instance;
    CanardTxQueue queue;
    CyphalTransportCan _transport;
    uint8_t _base[HEAP_SIZE] __attribute__ ((aligned (O1HEAP_ALIGNMENT)));

    HeartbeatPublisher heartbeat_pub;
    PortListPublisher port_list_pub;
    uint32_t error_counter = 0;
};

#endif  // CYPHAL_NODE_HPP_