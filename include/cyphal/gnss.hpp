/// This software is distributed under the terms of the MIT License.
/// Copyright (c) 2022 Dmitry Ponomarev.
/// Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

/**
 * @file gnss.hpp
 * @author d.ponomarev
 * @date Dec 28, 2021
 */

#ifndef CYPHAL_GNSS_HPP_
#define CYPHAL_GNSS_HPP_

#include "cyphal_node.hpp"
#include "reg/udral/physics/kinematics/geodetic/PointStateVarTs_0_1.h"
#include "uavcan/si/sample/angle/Scalar_1_0.h"
#include "uavcan/primitive/scalar/Integer16_1_0.h"

struct GeodeticPointPublisher: public CyphalPublisher {
    GeodeticPointPublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1& msg);
};

struct AngleScalarPublisher: public CyphalPublisher {
    AngleScalarPublisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_si_sample_angle_Scalar_1_0& msg);
};

struct PrimitiveInteger16Publisher: public CyphalPublisher {
    PrimitiveInteger16Publisher(CyphalNode* driver_, CanardPortID port_id) : CyphalPublisher(driver_, port_id) {};
    void publish(const uavcan_primitive_scalar_Integer16_1_0& msg);
};

#endif  // CYPHAL_GNSS_HPP_