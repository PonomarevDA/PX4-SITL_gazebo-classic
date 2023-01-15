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

// #include <vector>
// #include <regex>
// #include <thread>
// #include <mutex>
// #include <condition_variable>
// #include <deque>
// #include <atomic>
// #include <chrono>
// #include <memory>
// #include <sstream>
// #include <cassert>
// #include <stdexcept>
// #include <boost/asio.hpp>
// #include <boost/bind.hpp>
// #include <boost/shared_array.hpp>
// #include <boost/system/system_error.hpp>

// #include <iostream>
// #include <random>
// #include <stdio.h>
// #include <math.h>
// #include <cstdint>
// #include <cstdlib>
// #include <string>
// #include <sys/socket.h>
// #include <netinet/in.h>

// #include <Eigen/Eigen>
// #include<Eigen/StdVector>

// #include <development/mavlink.h>
// #include "msgbuffer.h"

#include "canard.h"
#include "cyphal/cyphal_node.hpp"


class CyphalInterface {
public:
    CyphalInterface();
    void process();
    ~CyphalInterface();
private:
    CyphalNode _node;
};
