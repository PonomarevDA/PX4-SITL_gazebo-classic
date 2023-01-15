/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file cyphal_node.cpp
 * @author d.ponomarev
 * @date Dec 21, 2022
 */

#include "cyphal/cyphal_node.hpp"
#include <iostream>
#include <string.h>
#include <chrono>
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/Health_1_0.h"

///< wrappers
static void* memAllocate(CanardInstance* const canard, const size_t amount);
static void memFree(CanardInstance* const canard, void* const pointer);
static uint32_t getCurrentMicroseconds();

O1HeapInstance* allocator;

bool CyphalNode::init() {
    if (!_transport.init(1000000, 0)) {
        return -1;
    }

    allocator = o1heapInit(_base, HEAP_SIZE);
    if (NULL == allocator) {
        return -1;
    }
    
    canard_instance = canardInit(&memAllocate, &memFree);
    canard_instance.node_id = (CanardNodeID)2;
    queue = canardTxInit(TX_QUEUE_FRAME_SIZE, CANARD_MTU_CAN_CLASSIC);

    return true;
}

void CyphalNode::process() {
    // 1. spin recv
    CanardFrame rx_frame;
    if (_transport.receive(&rx_frame)) {
        spinReceivedFrame(getCurrentMicroseconds() * 1000, &rx_frame);
        std::cout << "Receive smth" << std::endl;
        std::cout << std::flush;
    }

    // 2. spin application
    static uint32_t next_pub_time_ms = 50;  // a little timeout to initialize the internal state
    if (next_pub_time_ms < getCurrentMicroseconds()) {
        next_pub_time_ms += 500;
        uavcan_node_Heartbeat_1_0 heartbeat_msg;
        heartbeat_msg.health.value = uavcan_node_Health_1_0_NOMINAL;
        heartbeat_msg.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
        heartbeat_msg.uptime = getCurrentMicroseconds() / 1000;
        heartbeat_msg.vendor_specific_status_code = 0;
        heartbeat_pub.publish(heartbeat_msg);

        port_list_pub.publish();
    }

    // 3. spin tx
    spinTransmit();
}

int32_t CyphalNode::push(CanardTransferMetadata* metadata, size_t payload_size, const void *payload) {
    if (metadata->port_id == 0) {
        return 0;
    }
    auto res = canardTxPush(&queue, &canard_instance, 0, metadata, payload_size, payload);
    metadata->transfer_id++;
    return res;
}

int8_t CyphalNode::subscribe(CyphalSubscriber* sub_info, size_t size, CanardTransferKind kind) {
    if (_sub_num >= MAX_SUB_NUM) {
        return -1;
    }

    _sub_info[_sub_num] = sub_info;

    int8_t res = canardRxSubscribe(&canard_instance,
                                   kind,
                                   _sub_info[_sub_num]->port_id,
                                   size,
                                   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                   &_sub_info[_sub_num]->subscription);

    _sub_num++;

    return res;
}

void CyphalNode::processReceivedTransfer(const uint8_t redundant_interface_index,
                                         const CanardRxTransfer& transfer) {
    const CanardPortID PORT_ID = transfer.metadata.port_id;
    if (PORT_ID == uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_) {
        asm("NOP");
    }

    for (size_t sub_idx = 0; sub_idx < _sub_num; sub_idx++) {
        if (PORT_ID == _sub_info[sub_idx]->port_id) {
            _sub_info[sub_idx]->callback(transfer);
        }
    }
}

void CyphalNode::spinTransmit() {
    for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek(&queue)) != NULL;) {
        if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > getCurrentMicroseconds())) {
            if (!_transport.transmit(getCurrentMicroseconds(), ti)) {
                break;
            }
        }
        canard_instance.memory_free(&canard_instance, canardTxPop(&queue, ti));
    }
}

int8_t CyphalNode::subscribeApplication() {

}

// rx_timestamp_usec - When the frame was received, in microseconds.
// received_frame - The CAN frame received from the bus.
void CyphalNode::spinReceivedFrame(const CanardMicrosecond rx_timestamp_usec,
                                   const CanardFrame* const received_frame) {
    CanardRxTransfer transfer;
    const int8_t result = canardRxAccept(&canard_instance,
                                         rx_timestamp_usec,
                                         received_frame,
                                         0,
                                         &transfer,
                                         NULL);
    if (result < 0) {
        error_counter++;
    } else if (result == 1) {
        processReceivedTransfer(0, transfer);
        canard_instance.memory_free(&canard_instance, transfer.payload);
    }
}

void* memAllocate(CanardInstance* const canard, const size_t amount) {
    (void) canard;
    return o1heapAllocate(allocator, amount);
}
void memFree(CanardInstance* const canard, void* const pointer) {
    (void) canard;
    o1heapFree(allocator, pointer);
}
uint32_t getCurrentMicroseconds() {
    static auto time_start = std::chrono::steady_clock::now();
    auto time_now = std::chrono::steady_clock::now();
    auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start).count();
    return elapsed_time_ms;
}
