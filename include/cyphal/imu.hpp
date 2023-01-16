/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file imu.hpp
 * @author d.ponomarev
 * @date Dec 28, 2021
 */

#ifndef CYPHAL_IMU_HPP_
#define CYPHAL_IMU_HPP_

#include "cyphal_node.hpp"
#include "uavcan/si/sample/acceleration/Vector3_1_0.h"
#include "uavcan/si/sample/angular_velocity/Vector3_1_0.h"

struct AccelerometerPublisher: public CyphalPublisher {
    AccelerometerPublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_si_sample_acceleration_Vector3_1_0& msg);
};

struct GyroscopePublisher: public CyphalPublisher {
    GyroscopePublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_si_sample_angular_velocity_Vector3_1_0& msg);
};

#endif  // CYPHAL_IMU_HPP_