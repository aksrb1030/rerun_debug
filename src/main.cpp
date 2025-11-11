#include <rerun.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

namespace {
struct SensorSample {
    std::array<float, 3> position;
    float temperature_c;
    std::array<float, 3> velocity;
};

std::vector<SensorSample> generate_samples(std::mt19937 &rng, float time_seconds) {
    std::normal_distribution<float> noise(0.0f, 0.05f);

    std::vector<SensorSample> samples;
    samples.reserve(3);

    const std::array<std::array<float, 3>, 3> base_positions = {
        std::array<float, 3>{0.0f, 0.0f, 0.0f},
        std::array<float, 3>{1.0f, 0.2f, 0.1f},
        std::array<float, 3>{-0.6f, 0.9f, -0.2f}
    };

    for (std::size_t i = 0; i < base_positions.size(); ++i) {
        const float phase = time_seconds * 0.5f + static_cast<float>(i);
        const float temp = 22.0f + 4.0f * std::sin(phase) + noise(rng);

        const std::array<float, 3> vel = {
            std::cos(phase) * 0.25f + noise(rng),
            std::sin(phase * 0.8f) * 0.25f + noise(rng),
            std::cos(phase * 0.6f) * 0.25f + noise(rng)
        };

        std::array<float, 3> position = base_positions[i];
        for (std::size_t axis = 0; axis < 3; ++axis) {
            position[axis] += vel[axis] * 0.2f;
        }

        samples.push_back(SensorSample{position, temp, vel});
    }

    return samples;
}
}

int main() {
    try {
        auto rec = rerun::RecordingStream("sensor_visualizer");
        rec.spawn().throw_on_err();

        std::mt19937 rng{std::random_device{}()};
        const auto start = std::chrono::steady_clock::now();

        for (std::uint32_t frame = 0; frame < 300; ++frame) {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(now - start);

            const auto samples = generate_samples(rng, elapsed.count());

            std::vector<rerun::Vec3D> positions;
            std::vector<float> radii;
            std::vector<rerun::Rgba32> colors;
            std::vector<rerun::Vec3D> velocities;
            std::ostringstream temp_log;
            temp_log << std::fixed << std::setprecision(2);

            for (std::size_t i = 0; i < samples.size(); ++i) {
                const auto &sample = samples[i];

                positions.emplace_back(sample.position[0], sample.position[1], sample.position[2]);
                radii.push_back(0.06f);

                const float normalized = std::clamp((sample.temperature_c - 20.0f) / 8.0f, 0.0f, 1.0f);
                const auto color = rerun::Rgba32::from_unmultiplied_rgba(
                    static_cast<std::uint8_t>(normalized * 255.0f),
                    static_cast<std::uint8_t>((1.0f - normalized) * 255.0f),
                    180,
                    255
                );
                colors.push_back(color);
                velocities.emplace_back(sample.velocity[0], sample.velocity[1], sample.velocity[2]);

                temp_log << "Sensor " << i
                         << ": " << sample.temperature_c << " °C\n";
            }

            rec.set_time_sequence("frame", static_cast<int64_t>(frame));
            rec.set_time_seconds("time", elapsed.count());

            rec.log("sensors/positions", rerun::Points3D(positions)
                                             .with_radii(radii)
                                             .with_colors(colors));

            rec.log("sensors/velocity", rerun::Arrows3D(positions, velocities));
            rec.log("sensors/temperatures", rerun::TextLog(temp_log.str()));

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception &err) {
        std::cerr << "Failed to run sensor visualizer: " << err.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
