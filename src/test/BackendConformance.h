#pragma once

#include <functional>
#include <memory>
#include <string_view>

#include "machine/BackendRuntime.h"

namespace ngc::test {
    struct BackendConformanceTarget {
        std::string_view name;
        std::function<std::unique_ptr<BackendRuntime>()> createRuntime;
        std::function<TriggeredJointMove()> makeTriggeredJointMove;
    };

    void runBackendConformanceSuite(const BackendConformanceTarget &target);
}
