/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Cyphal interface
 *
 * @author Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#include "cyphal_interface.h"
#include <iostream>

CyphalInterface::CyphalInterface() : _geodetic_point_pub(&_node, 65535),
                                     _sats_pub(&_node, 65535),
                                     _status_pub(&_node, 65535),
                                     _pdop_pub(&_node, 65535),
                                     _baro_pressure_pub(&_node, 65535),
                                     _baro_temperature_pub(&_node, 65535) {
    std::cout << "CyphalInterface" << std::endl;
    std::cout << std::flush;
    _node.init();
}

void CyphalInterface::process() {
    _node.process();
}

void CyphalInterface::SendGpsMessages(const SensorData::Gps &data) {
    reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1 point_state;
    _geodetic_point_pub.setPortId(2000);
    _geodetic_point_pub.publish(point_state);

    uavcan_primitive_scalar_Integer16_1_0 sats;
    _sats_pub.setPortId(2001);
    _sats_pub.publish(sats);

    uavcan_primitive_scalar_Integer16_1_0 status;
    _status_pub.setPortId(2002);
    _status_pub.publish(status);

    uavcan_primitive_scalar_Integer16_1_0 pdop;
    _pdop_pub.setPortId(2003);
    _pdop_pub.publish(pdop);
}

void CyphalInterface::UpdateBarometer(const SensorData::Barometer &data, const int id) {
    static auto next_pub_time = std::chrono::steady_clock::now();
    auto time_now = std::chrono::steady_clock::now();

    if (time_now < next_pub_time) {
        return;
    }
    next_pub_time = time_now + std::chrono::milliseconds(100);

    uavcan_si_sample_pressure_Scalar_1_0 pressure;
    _baro_pressure_pub.setPortId(2010);
    _baro_pressure_pub.publish(pressure);

    uavcan_si_sample_temperature_Scalar_1_0 temperature;
    _baro_temperature_pub.setPortId(2011);
    _baro_temperature_pub.publish(temperature);
}

void CyphalInterface::UpdateAirspeed(const SensorData::Airspeed &data, const int id) {

}

void CyphalInterface::UpdateIMU(const SensorData::Imu &data, const int id) {

}

void CyphalInterface::UpdateMag(const SensorData::Magnetometer &data, const int id) {

}

CyphalInterface::~CyphalInterface() {
    std::cout << "~CyphalInterface" << std::endl;
    std::cout << std::flush;
}
