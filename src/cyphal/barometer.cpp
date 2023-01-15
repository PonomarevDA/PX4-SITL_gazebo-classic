/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file barometer.cpp
 * @author d.ponomarev
 * @date Aug 12, 2022
 */

#include "cyphal/barometer.hpp"
#include <iostream>
#include <string.h>
#include <chrono>

void BaroPressurePublisher::publish(const uavcan_si_sample_pressure_Scalar_1_0& msg) {
    static uint8_t buffer[uavcan_si_sample_pressure_Scalar_1_0_EXTENT_BYTES_];
    size_t buffer_size = uavcan_si_sample_pressure_Scalar_1_0_EXTENT_BYTES_;
    int32_t result = uavcan_si_sample_pressure_Scalar_1_0_serialize_(&msg, buffer, &buffer_size);
    if (NUNAVUT_SUCCESS == result) {
        driver->push(&transfer_metadata, buffer_size, buffer);
    }
}

void BaroTemperaturePublisher::publish(const uavcan_si_sample_temperature_Scalar_1_0& msg) {
    static uint8_t buffer[uavcan_si_sample_temperature_Scalar_1_0_EXTENT_BYTES_];
    size_t buffer_size = uavcan_si_sample_temperature_Scalar_1_0_EXTENT_BYTES_;
    int32_t result = uavcan_si_sample_temperature_Scalar_1_0_serialize_(&msg, buffer, &buffer_size);
    if (NUNAVUT_SUCCESS == result) {
        driver->push(&transfer_metadata, buffer_size, buffer);
    }
}
