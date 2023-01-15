/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file magnetometer.hpp
 * @author d.ponomarev
 * @date Dec 28, 2021
 */

#ifndef CYPHAL_MAGNETOMETER_HPP_
#define CYPHAL_MAGNETOMETER_HPP_

#include "cyphal_node.hpp"
#include "uavcan/si/sample/magnetic_field_strength/Vector3_1_0.h"

struct MagneticFieldPublisher: public CyphalPublisher {
    MagneticFieldPublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_si_sample_magnetic_field_strength_Vector3_1_0& msg);
};

#endif  // CYPHAL_MAGNETOMETER_HPP_