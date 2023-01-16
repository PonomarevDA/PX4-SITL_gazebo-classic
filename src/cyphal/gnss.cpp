/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file gnss.cpp
 * @author d.ponomarev
 * @date Aug 12, 2022
 */

#include "cyphal/gnss.hpp"
#include <iostream>
#include <string.h>
#include <chrono>

void GeodeticPointPublisher::publish(const reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1& msg) {
    uint8_t buffer[reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_EXTENT_BYTES_];
    size_t buffer_size = reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_EXTENT_BYTES_;
    int32_t result = reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_serialize_(&msg, buffer, &buffer_size);
    if (NUNAVUT_SUCCESS == result) {
        driver->push(&transfer_metadata, buffer_size, buffer);
    }
}

void AngleScalarPublisher::publish(const uavcan_si_sample_angle_Scalar_1_0& msg) {
    uint8_t buffer[uavcan_si_sample_angle_Scalar_1_0_EXTENT_BYTES_];
    size_t buffer_size = uavcan_si_sample_angle_Scalar_1_0_EXTENT_BYTES_;
    int32_t result = uavcan_si_sample_angle_Scalar_1_0_serialize_(&msg, buffer, &buffer_size);
    if (NUNAVUT_SUCCESS == result) {
        driver->push(&transfer_metadata, buffer_size, buffer);
    }
}

void PrimitiveInteger16Publisher::publish(const uavcan_primitive_scalar_Integer16_1_0& msg) {
    uint8_t buffer[uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_];
    size_t buffer_size = uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_;
    int32_t result = uavcan_primitive_scalar_Integer16_1_0_serialize_(&msg, buffer, &buffer_size);
    if (NUNAVUT_SUCCESS == result) {
        driver->push(&transfer_metadata, buffer_size, buffer);
    }
}