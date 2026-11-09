// Offline oracle for protocol.rs tests. SDK commit:
// 0803d033b530fcfa0e6fa9236de68c2773fc9402 (unitreerobotics/r1-7a_sdk).
// Build against its original MotorProtocol.cpp and include directory:
// clang++ -std=c++17 -include sdk_compile_shim.h -I "$SDK/thirdparty/motor_controller/include" \
//   generate_protocol_golden.cpp "$SDK/thirdparty/motor_controller/src/MotorProtocol.cpp" \
//   -o /tmp/r17-golden && /tmp/r17-golden > protocol-golden.json
// Expose only the SDK CRC routine to build synthetic feedback; no SDK edits.
#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#define private public
#include "MotorProtocol.h"
#undef private
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>

static void hex(const std::vector<uint8_t>& bytes) {
    std::cout << '"' << std::hex << std::setfill('0');
    for (auto byte : bytes) std::cout << std::setw(2) << int(byte);
    std::cout << std::dec << '"';
}

template<class Raw>
static float boundary(double physical, double wire_bound, Raw raw, double lo, double hi) {
    float v = float(std::copysign(std::min(std::fabs(physical), std::fabs(wire_bound)), physical));
    while (raw(v) < lo || raw(v) > hi) v = std::nextafter(v, 0.f);
    return v;
}

int main() {
    MotorProtocol p;
    std::cout << std::setprecision(17);
    std::cout << "{\"sdk_commit\":\"0803d033b530fcfa0e6fa9236de68c2773fc9402\",\"control\":[";
    bool comma = false;
    for (uint8_t id : {uint8_t(0), uint8_t(2), uint8_t(5)}) {
        // Largest float32 values inside BOTH the SDK physical clamps and raw
        // signed ranges. Rounding INT_MAX to float32 can cross the raw limit.
        double r = MotorProtocol::gearRatio(id);
        for (int sign : {-1, 0, 1}) {
            const double i32edge = sign < 0 ? 2147483648.0 : 2147483647.0;
            const double i16edge = sign < 0 ? 32768.0 : 32767.0;
            float q = boundary(sign * (sign < 0 ? 32508.539391f : 32508.539376f),
                i32edge * 2 * MotorProtocol::M_PI / (r * 32768),
                [r](float v) { return std::round(v * r * 32768.0 / MotorProtocol::M_PI / 2.0); }, INT32_MIN, INT32_MAX);
            float dq = boundary(sign * (sign < 0 ? 253.972964f : 253.965213f),
                i16edge * 2 * MotorProtocol::M_PI / (r * 64),
                [r](float v) { return std::round(v * r * 64.0 / MotorProtocol::M_PI / 2.0); }, INT16_MIN, INT16_MAX);
            float tau = boundary(sign * (sign < 0 ? 162.133333f : 162.128385f), i16edge * r / 2560,
                [r](float v) { return std::round(v / r * 2560.0); }, INT16_MIN, INT16_MAX);
            float kp = boundary(410.725243f, 32767.0 * r * r / 12800,
                [r](float v) { return std::round(v / (r*r) * 12800.0); }, INT16_MIN, INT16_MAX);
            float kd = boundary(102.681311f, 32767.0 * r * r / 51200,
                [r](float v) { return std::round(v / (r*r) * 51200.0); }, INT16_MIN, INT16_MAX);
            if (comma) std::cout << ',';
            comma = true;
            std::cout << "{\"id\":" << int(id) << ",\"q\":" << q << ",\"dq\":" << dq
                      << ",\"tau\":" << tau << ",\"kp\":" << kp << ",\"kd\":" << kd << ",\"packet\":";
            hex(p.buildControlPacket(id, 1, true, tau, dq, q, kp, kd));
            std::cout << '}';
        }
    }
    std::cout << "],\"disabled\":";
    hex(p.buildControlPacket(6, 0, false, 0, 0, 0, 0, 0));
    std::cout << ",\"clear_fault\":";
    hex(p.buildClearPacket(6));
    std::cout << ",\"feedback\":[";
    comma = false;
    for (uint8_t id : {uint8_t(0), uint8_t(2), uint8_t(5)}) {
        for (bool negative : {false, true}) {
            std::vector<uint8_t> frame(26, 0);
            frame[0] = 0xfc; frame[1] = 0xee; frame[2] = 0x90 | id;
            frame[3] = uint8_t(int8_t(-23)); frame[4] = 101; frame[5] = 97;
            int16_t t = negative ? INT16_MIN : INT16_MAX;
            int16_t v = negative ? INT16_MIN : INT16_MAX;
            int32_t q = negative ? INT32_MIN : INT32_MAX;
            uint32_t error = 0x80010002; uint16_t flags = 5u << 13;
            std::memcpy(frame.data()+6, &t, 2); std::memcpy(frame.data()+8, &v, 2);
            std::memcpy(frame.data()+10, &q, 4); std::memcpy(frame.data()+14, &error, 4);
            std::memcpy(frame.data()+18, &flags, 2);
            uint32_t crc = p.calculateCRC32(frame.data()+2, 20);
            std::memcpy(frame.data()+22, &crc, 4);
            auto fb = p.parseFeedbackPacket(frame);
            if (!fb) return 2;
            if (comma) std::cout << ',';
            comma = true;
            std::cout << "{\"packet\":"; hex(frame);
            std::cout << ",\"id\":" << int(fb->motor_id) << ",\"q\":" << fb->position
                      << ",\"dq\":" << fb->speed << ",\"tau\":" << fb->torque
                      << ",\"temperature\":" << int(fb->temp_driver) << ",\"voltage\":" << fb->voltage
                      << ",\"error\":" << fb->error_code << ",\"warning\":" << int(fb->warning_code)
                      << ",\"mode\":" << int(fb->mode) << ",\"timeout\":" << int(fb->timeout) << '}';
        }
    }
    std::cout << "]}\n";
}
