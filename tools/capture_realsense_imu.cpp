// Read-only SDK capture for the static gyro calibration tool. No EEPROM writes.
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

static double monotonic_seconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static std::string quoted(const std::string &s) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4)
                             << std::setfill('0') << static_cast<int>(c);
        else out << c;
    }
    out << '"';
    return out.str();
}

static void write_intrinsics(std::ostream &out, const rs2::stream_profile &profile) {
    out << "{\"fps\":" << profile.fps() << ",\"intrinsics\":";
    try {
        auto intr = profile.as<rs2::motion_stream_profile>().get_motion_intrinsics();
        out << "{\"data\":[";
        for (int row = 0; row < 3; ++row) {
            if (row) out << ',';
            out << '[';
            for (int col = 0; col < 4; ++col) {
                if (col) out << ',';
                out << intr.data[row][col];
            }
            out << ']';
        }
        out << "],\"noise_variances\":[";
        for (int i = 0; i < 3; ++i) { if (i) out << ','; out << intr.noise_variances[i]; }
        out << "],\"bias_variances\":[";
        for (int i = 0; i < 3; ++i) { if (i) out << ','; out << intr.bias_variances[i]; }
        out << "]}";
    } catch (const rs2::error &) {
        out << "null";
    }
    out << '}';
}

int main(int argc, char **argv) {
    if (argc != 5 && argc != 6) {
        std::cerr << "usage: capture_realsense_imu SECONDS DATA.csv META.json SERIAL_OR_EMPTY [START_TIMEOUT]\n";
        return 2;
    }
    try {
        double duration = std::stod(argv[1]);
        double startup = argc == 6 ? std::stod(argv[5]) : 15.0;
        if (!std::isfinite(duration) || duration < 1 || duration > 3600 ||
            !std::isfinite(startup) || startup < 1 || startup > 120)
            throw std::runtime_error("invalid capture duration/start timeout");
        rs2::context context;
        std::vector<rs2::device> candidates;
        for (auto dev : context.query_devices()) {
            if (std::string(argv[4]).empty() || dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == std::string(argv[4]))
                candidates.push_back(dev);
        }
        if (candidates.size() != 1)
            throw std::runtime_error("expected one camera; use --camera-serial when more than one is connected");
        auto &device = candidates[0];
        rs2::sensor motion;
        rs2::stream_profile gyro, accel;
        for (const auto &sensor : device.query_sensors()) {
            rs2::stream_profile g, a;
            for (auto p : sensor.get_stream_profiles()) {
                if (p.format() != RS2_FORMAT_MOTION_XYZ32F) continue;
                if (p.stream_type() == RS2_STREAM_GYRO &&
                    (!g || std::abs(p.fps() - 200) < std::abs(g.fps() - 200))) g = p;
                if (p.stream_type() == RS2_STREAM_ACCEL &&
                    (!a || std::abs(p.fps() - 100) < std::abs(a.fps() - 100))) a = p;
            }
            if (g && a) { motion = sensor; gyro = g; accel = a; break; }
        }
        if (!motion) throw std::runtime_error("camera has no gyro + accel motion sensor");
        std::ofstream meta(argv[3]);
        if (!meta) throw std::runtime_error("cannot open metadata output");
        meta << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "{\"name\":" << quoted(device.get_info(RS2_CAMERA_INFO_NAME))
             << ",\"serial\":" << quoted(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))
             << ",\"firmware\":" << quoted(device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION))
             << ",\"sdk_version\":" << quoted(RS2_API_VERSION_STR)
             << ",\"coordinate_frame\":\"realsense_sdk_motion_xyz\""
             << ",\"motion_correction_enabled\":";
        if (motion.supports(RS2_OPTION_ENABLE_MOTION_CORRECTION))
            meta << motion.get_option(RS2_OPTION_ENABLE_MOTION_CORRECTION);
        else meta << "null";
        meta << ",\"gyro\":"; write_intrinsics(meta, gyro);
        meta << ",\"accel\":"; write_intrinsics(meta, accel);
        meta << ",\"device_calibration_written\":false}\n";
        meta.close();
        std::ofstream data(argv[2]);
        if (!data) throw std::runtime_error("cannot open sample output");
        // Global timestamps are epoch milliseconds (~1.8e12). Twelve digits
        // would round 200 Hz samples onto a 10 ms grid. Preserve all doubles.
        data << std::setprecision(std::numeric_limits<double>::max_digits10)
             << "stream,host_s,stamp,frame_number,timestamp_domain,x,y,z\n";
        // Both the stream and callback-owned state outlive sensor.stop().
        std::mutex mutex;
        std::atomic<double> first{0}, last{0};
        std::atomic<bool> bad_frame{false};
        const double began = monotonic_seconds();
        bool opened = false, started = false;
        try {
            motion.open(std::vector<rs2::stream_profile>{gyro, accel}); opened = true;
            motion.start([&](rs2::frame frame) {
                try {
                    auto sample = frame.as<rs2::motion_frame>();
                    if (!sample) return;
                    auto value = sample.get_motion_data();
                    auto kind = sample.get_profile().stream_type();
                    if (kind != RS2_STREAM_GYRO && kind != RS2_STREAM_ACCEL) return;
                    const double now = monotonic_seconds();
                    std::lock_guard<std::mutex> lock(mutex);
                    if (!first.load()) first.store(now);
                    last.store(now);
                    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
                        bad_frame.store(true);
                    data << (kind == RS2_STREAM_GYRO ? "gyro" : "accel") << ',' << now << ','
                         << frame.get_timestamp() << ',' << frame.get_frame_number() << ','
                         << static_cast<int>(frame.get_frame_timestamp_domain()) << ','
                         << value.x << ',' << value.y << ',' << value.z << '\n';
                } catch (...) { bad_frame.store(true); }
            }); started = true;
            while (!first.load() || monotonic_seconds() - first.load() < duration) {
                double now = monotonic_seconds();
                if ((!first.load() && now - began > startup) ||
                    (last.load() && now - last.load() > 3))
                    throw std::runtime_error("camera IMU data timed out");
                if (bad_frame.load()) throw std::runtime_error("invalid camera motion frame");
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            motion.stop(); started = false;
            motion.close(); opened = false;
        } catch (...) {
            if (started) { try { motion.stop(); } catch (...) {} }
            if (opened) { try { motion.close(); } catch (...) {} }
            throw;
        }
        data.flush();
        if (!data) throw std::runtime_error("sample write failed");
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
