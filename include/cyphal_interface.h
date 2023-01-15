/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include "canard.h"
#include "cyphal/barometer.hpp"
#include "cyphal/cyphal_node.hpp"
#include "cyphal/magnetometer.hpp"
#include "cyphal/gnss.hpp"
#include "mavlink_interface.h"


class CyphalInterface {
public:
    CyphalInterface();
    ~CyphalInterface();
    void process();
    void SendGpsMessages(const SensorData::Gps &data);
    void UpdateBarometer(const SensorData::Barometer &data, const int id = 0);
    void UpdateAirspeed(const SensorData::Airspeed &data, const int id = 0);
    void UpdateIMU(const SensorData::Imu &data, const int id = 0);
    void UpdateMag(const SensorData::Magnetometer &data, const int id = 0);
private:
    CyphalNode _node;

    GeodeticPointPublisher _geodetic_point_pub;
    PrimitiveInteger16Publisher _sats_pub;
    PrimitiveInteger16Publisher _status_pub;
    PrimitiveInteger16Publisher _pdop_pub;

    BaroPressurePublisher _baro_pressure_pub;
    BaroTemperaturePublisher _baro_temperature_pub;

    MagneticFieldPublisher _mag_pub;
};
