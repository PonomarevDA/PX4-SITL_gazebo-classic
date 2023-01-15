/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file barometer.hpp
 * @author d.ponomarev
 * @date Dec 28, 2021
 */

#ifndef CYPHAL_BAROMETER_HPP_
#define CYPHAL_BAROMETER_HPP_

#include "cyphal_node.hpp"
#include "uavcan/si/sample/pressure/Scalar_1_0.h"
#include "uavcan/si/sample/temperature/Scalar_1_0.h"

struct BaroPressurePublisher: public CyphalPublisher {
    BaroPressurePublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_si_sample_pressure_Scalar_1_0& msg);
};

struct BaroTemperaturePublisher: public CyphalPublisher {
    BaroTemperaturePublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_si_sample_temperature_Scalar_1_0& msg);
};

#endif  // CYPHAL_BAROMETER_HPP_