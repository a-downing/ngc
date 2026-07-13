#include "machine/MachineConfiguration.h"

#include <cmath>
#include <format>
#include <string_view>

#include <toml++/toml.hpp>

namespace ngc {
    namespace {
        std::expected<double, std::string> positiveNumber(
            const toml::table &table, const std::string_view name, const std::filesystem::path &path) {
            const auto value = table[name].value<double>();
            if(!value || !std::isfinite(*value) || *value <= 0.0)
                return std::unexpected(std::format("{}: '{}' must be a finite number greater than zero",
                                                   path.string(), name));
            return *value;
        }
    }

    std::expected<MachineConfiguration, std::string>
    loadMachineConfiguration(const std::filesystem::path &path) {
        try {
            const auto document = toml::parse_file(path.string());
            const auto *machine = document["machine"].as_table();
            const auto *trajectory = document["trajectory"].as_table();
            const auto *simulation = document["simulation"].as_table();
            if(!machine) return std::unexpected(std::format("{}: missing [machine] table", path.string()));
            if(!trajectory) return std::unexpected(std::format("{}: missing [trajectory] table", path.string()));
            if(!simulation) return std::unexpected(std::format("{}: missing [simulation] table", path.string()));

            const auto units = (*machine)["units"].value<std::string>();
            if(!units || (*units != "inch" && *units != "mm"))
                return std::unexpected(std::format("{}: machine.units must be 'inch' or 'mm'", path.string()));

            const auto acceleration = positiveNumber(*trajectory, "path_acceleration", path);
            const auto jerk = positiveNumber(*trajectory, "path_jerk", path);
            const auto rapidVelocity = positiveNumber(*trajectory, "rapid_velocity", path);
            const auto chordTolerance = positiveNumber(*trajectory, "arc_chord_tolerance", path);
            const auto servoPeriod = positiveNumber(*simulation, "servo_period", path);
            const auto schedulerPeriod = positiveNumber(*simulation, "scheduler_period", path);
            if(!acceleration) return std::unexpected(acceleration.error());
            if(!jerk) return std::unexpected(jerk.error());
            if(!rapidVelocity) return std::unexpected(rapidVelocity.error());
            if(!chordTolerance) return std::unexpected(chordTolerance.error());
            if(!servoPeriod) return std::unexpected(servoPeriod.error());
            if(!schedulerPeriod) return std::unexpected(schedulerPeriod.error());
            const auto servoTicksPerSchedulerPeriod = *schedulerPeriod / *servoPeriod;
            if(servoTicksPerSchedulerPeriod < 1.0
               || std::abs(servoTicksPerSchedulerPeriod - std::round(servoTicksPerSchedulerPeriod)) > 1e-9)
                return std::unexpected(std::format(
                    "{}: simulation.scheduler_period must be an integer multiple of simulation.servo_period",
                    path.string()));

            MachineConfiguration result;
            result.unit = *units == "inch" ? Machine::Unit::Inch : Machine::Unit::Millimeter;
            result.trajectory.pathAcceleration = *acceleration;
            result.trajectory.pathJerk = *jerk;
            // Canonical G-code feed values, including the rapid sentinel replacement,
            // are currently expressed per minute at the planner boundary.
            result.trajectory.rapidSpeed = *rapidVelocity * 60.0;
            result.trajectory.arcChordTolerance = *chordTolerance;
            result.simulation.servoPeriod = *servoPeriod;
            result.simulation.schedulerPeriod = *schedulerPeriod;
            return result;
        } catch(const toml::parse_error &error) {
            return std::unexpected(std::format("{}:{}:{}: {}", path.string(), error.source().begin.line,
                                               error.source().begin.column, error.description()));
        }
    }
}
