#include "Application.h"
#include "PreviewSpline.h"

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <print>
#include <string>
#include <cmath>
#include <cstdint>
#include <ranges>
#include <numbers>
#include <limits>
#include <optional>
#include <utility>
#include <iterator>
#include <sstream>
#include <unordered_set>

#include <expected>

#ifdef _WIN32
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/ext/vector_double3.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/ext/quaternion_common.hpp>

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_stdlib.h"
#include "gui/imgui_custom.h"

#include "utils.h"
#include "SimulationWorker.h"
#include "Worker.h"

#include "machine/ToolTable.h"
#include "machine/MachineCommand.h"

#include "gui/tool_table_strings_t.h"

class ApplicationImpl final {
    GLFWwindow *m_window;
    std::filesystem::path m_path = std::filesystem::absolute(".").lexically_normal();
    ngc::ToolTable m_tools;
    std::vector<tool_table_strings_t> m_toolStrings;
    std::vector<std::tuple<std::string, std::string>> m_autoloadSource;
    std::vector<std::tuple<std::string, std::string>> m_programSource;
    std::vector<std::string> m_mainProgramLines;
    std::vector<std::string> m_mdiHistory;
    std::string m_mdiInput;
    static constexpr std::string_view MDI_SOURCE_NAME = "<MDI>";

    enum class ProgramPaneMode { Edit, Compiled };
    ProgramPaneMode m_programPaneMode = ProgramPaneMode::Edit;
    bool m_programDirty = false;
    bool m_previewAfterCompile = false;
    float m_programPaneWidth = 440.0f;
    float m_statusBarHeight = 170.0f;
    float m_toolbarHeight = 42.0f;
    ImVec4 m_viewportRect { 0, 0, 0, 0 };

    // windows
    bool m_enableOpenDialog = false;
    bool m_enableProgramWindow = false; // Legacy window is no longer opened; retained until its renderer is removed.
    bool m_enableMemoryWindow = false;
    bool m_enableToolWindow = false;
    bool m_enableErrorWindow = false;

    std::string m_errorMessage;

    Worker m_worker{};
    SimulationWorker m_simulation{};
    double m_simulationRate = 1.0;
    double m_simulatedRapidSpeed = 100.0;

    double m_time = 0.0;
    double m_dt = 0.0;
    double m_mouseX = 0.0;
    double m_mouseY = 0.0;
    double m_mouseDX = 0.0;
    double m_mouseDY = 0.0;
    double m_scrollDelta = 0.0;

    glm::dquat m_modelOrientation { 1.0, 0.0, 0.0, 0.0 };
    glm::dvec2 m_viewPan { 0.0, 0.0 };
    double m_viewHalfHeight = 6.0;

    struct PreviewRenderCache {
        std::optional<std::uint64_t> revision;
        std::vector<glm::dvec3> feedLines;
        std::vector<glm::dvec3> rapidLines;
        std::vector<glm::dvec3> g53FeedLines;
        std::vector<glm::dvec3> g53RapidLines;
        std::vector<glm::dvec3> arcLines;
        std::vector<glm::dvec3> probeLines;
        std::vector<glm::dvec3> g64SplineLines;
        std::vector<glm::dvec3> g64ClusterSplineLines;
        std::vector<glm::dvec3> g64ControlPolygon;
        std::vector<glm::dvec3> g64ControlPoints;
        std::vector<glm::dvec3> darkPoints;
        std::vector<glm::dvec3> lightPoints;
    } m_previewRenderCache;
    double m_sceneScale = 1.0;
    glm::dvec3 m_modelPivot = { 0.0, 0.0, 0.0 };

    static constexpr double MACHINE_TRIAD_LENGTH = 2.0;
    static constexpr double WORK_TRIAD_LENGTH = 1.0;

    static glm::dvec3 displayOffset(const glm::dvec3 &offset) {
        return { offset.x, -offset.y, offset.z };
    }

    static std::expected<std::vector<std::tuple<std::string, std::string>>, std::string> readAutoloadSources() {
        std::vector<std::tuple<std::string, std::string>> programs;
        std::error_code error;
        std::filesystem::directory_iterator iterator("autoload", error);
        for(; !error && iterator != std::filesystem::directory_iterator(); iterator.increment(error)) {
            const auto &entry = *iterator;
            std::error_code typeError;
            if(!entry.is_regular_file(typeError)) {
                if(typeError) error = typeError;
                continue;
            }

            const auto result = ngc::readFile(entry.path());
            if(!result) return std::unexpected(std::string(result.error().what()));
            programs.emplace_back(*result, entry.path().string());
        }

        if(error) return std::unexpected(std::format("failed to read autoload directory: {}", error.message()));
        return programs;
    }

    void drawToolWireframe(const ngc::ToolPose &tool) const {
        const glm::dvec3 top {
            tool.spindlePosition.x,
            tool.spindlePosition.y,
            tool.spindlePosition.z,
        };
        const glm::dvec3 tip {
            tool.tipPosition.x,
            tool.tipPosition.y,
            tool.tipPosition.z,
        };
        const auto axis = tip - top;
        if(tool.geometry.number == 0 || glm::length2(axis) < 1e-18) return;

        const auto axisUnit = glm::normalize(axis);
        const auto cameraDirection = displayOffset(glm::inverse(m_modelOrientation) * glm::dvec3(0.0, 1.0, 0.0));
        auto radial = glm::cross(axisUnit, cameraDirection);
        if(glm::length2(radial) < 1e-18) {
            radial = glm::cross(axisUnit, glm::dvec3(1.0, 0.0, 0.0));
        }
        radial = glm::normalize(radial);
        const auto radius = std::max(tool.geometry.diameter * 0.5, 0.001);
        const auto edge0 = top + radial * radius;
        const auto edge1 = top - radial * radius;

        auto circleBasis0 = glm::cross(axisUnit, glm::dvec3(0.0, 0.0, 1.0));
        if(glm::length2(circleBasis0) < 1e-18) {
            circleBasis0 = glm::cross(axisUnit, glm::dvec3(0.0, 1.0, 0.0));
        }
        circleBasis0 = glm::normalize(circleBasis0);
        const auto circleBasis1 = glm::normalize(glm::cross(axisUnit, circleBasis0));

        glDisable(GL_DEPTH_TEST);
        glLineWidth(2.0f);
        glColor3f(1.0f, 0.85f, 0.05f);
        glBegin(GL_LINES);
        glVertex3dv(glm::value_ptr(edge0));
        glVertex3dv(glm::value_ptr(tip));
        glVertex3dv(glm::value_ptr(edge1));
        glVertex3dv(glm::value_ptr(tip));
        glEnd();

        constexpr int CIRCLE_SEGMENTS = 48;
        glBegin(GL_LINE_LOOP);
        for(int segment = 0; segment < CIRCLE_SEGMENTS; segment++) {
            const auto angle = 2.0 * std::numbers::pi * static_cast<double>(segment) / CIRCLE_SEGMENTS;
            const auto point = top + radius * (std::cos(angle) * circleBasis0 + std::sin(angle) * circleBasis1);
            glVertex3dv(glm::value_ptr(point));
        }
        glEnd();

        glEnable(GL_POINT_SMOOTH);
        glPointSize(5.0f);
        glBegin(GL_POINTS);
        glVertex3dv(glm::value_ptr(top));
        glEnd();
        glPointSize(1.0f);
        glLineWidth(1.0f);
    }

public:
    ApplicationImpl() = delete;
    explicit ApplicationImpl(GLFWwindow *window) : m_window(window) { }

    void init() {
        auto result = m_tools.load();

        if(!result) {
            m_errorMessage = "failed to load tool table";
            m_enableErrorWindow = true;
        }

        auto autoload = readAutoloadSources();
        if(!autoload) {
            m_errorMessage = autoload.error();
            m_enableErrorWindow = true;
        } else {
            m_autoloadSource = std::move(*autoload);
        }
    }

    void preRender() {
        m_dt = ImGui::GetTime() - m_time;
        m_time = ImGui::GetTime();
        auto pos = ImGui::GetMousePos();
        m_mouseDX = pos.x - m_mouseX;
        m_mouseDY = pos.y - m_mouseY;
        m_mouseX = pos.x;
        m_mouseY = pos.y;
    }

    void addScroll(const double delta) {
        m_scrollDelta += delta;
    }

    void terminate() {
        m_simulation.join();
        m_worker.join();
    }

    void initToolTableStrings() {
        m_toolStrings.clear();

        for(const auto &[num, tool] : m_tools) {
            m_toolStrings.emplace_back(tool_table_strings_t::from(tool));
        }
    }

    void saveToolTableStrings() {
        ngc::ToolTable tools;

        for(const auto &toolStrings : m_toolStrings) {
            auto tool = toolStrings.to();

            if(!tool) {
                m_errorMessage = std::format("tool #{}: {}", toolStrings.number, tool.error());
                m_enableErrorWindow = true;
                return;
            }

            if(tools.get(tool->number)) {
                m_errorMessage = std::format("duplicate tool number {}", tool->number);
                m_enableErrorWindow = true;
                return;
            }

            tools.set(tool->number, *tool);
        }

        auto result = tools.save();

        if(!result) {
            m_errorMessage = result.error();
            m_enableErrorWindow = true;
            return;
        }

        if(!m_worker.setToolTable(tools)) {
            m_errorMessage = "tool table was saved, but cannot be applied while the worker is busy";
            m_enableErrorWindow = true;
            return;
        }

        m_tools = std::move(tools);
        initToolTableStrings();
    }

    void reloadToolTable() {
        const auto result = m_tools.load();

        if(!result) {
            m_errorMessage = result.error();
            m_enableErrorWindow = true;
            return;
        }

        if(!m_worker.setToolTable(m_tools)) {
            m_errorMessage = "tool table cannot be reloaded while the worker is busy";
            m_enableErrorWindow = true;
            return;
        }

        initToolTableStrings();
    }

    bool noWindowHasFocus() {
        auto ctx = ImGui::GetCurrentContext();
        return ctx->NavWindow == nullptr && ctx->MovingWindow == nullptr && ctx->WheelingWindow == nullptr;
    }

    void fitToolpathToView(const int width, const int height) {
        glm::dvec3 minimum(std::numeric_limits<double>::max());
        glm::dvec3 maximum(std::numeric_limits<double>::lowest());
        bool hasGeometry = false;

        const auto include = [&](const glm::dvec3 &point) {
            minimum = glm::min(minimum, point);
            maximum = glm::max(maximum, point);
            hasGeometry = true;
        };

        m_worker.lock([&] {
            m_worker.toolpath().foreachCommand([&](const ngc::MachineCommand &command) {
                std::visit([&](const auto &value) {
                    using T = std::decay_t<decltype(value)>;

                    if constexpr(std::same_as<T, ngc::MoveLine>) {
                        include({ value.from().x, value.from().y, value.from().z });
                        include({ value.to().x, value.to().y, value.to().z });
                    } else if constexpr(std::same_as<T, ngc::ProbeMove>) {
                        include({ value.from().x, value.from().y, value.from().z });
                        include({ value.target().x, value.target().y, value.target().z });
                    } else if constexpr(std::same_as<T, ngc::MoveArc>) {
                        const glm::dvec3 from { value.from().x, value.from().y, value.from().z };
                        const glm::dvec3 to { value.to().x, value.to().y, value.to().z };
                        const glm::dvec3 center { value.center().x, value.center().y, value.center().z };
                        const auto radius = std::max(glm::length(from - center), glm::length(to - center));
                        include(from);
                        include(to);
                        include(center - glm::dvec3(radius));
                        include(center + glm::dvec3(radius));
                    }
                }, command);
            });
        });

        if(!hasGeometry) {
            minimum = { -1.0, -1.0, -1.0 };
            maximum = { 1.0, 1.0, 1.0 };
        }

        m_modelPivot = (minimum + maximum) * 0.5;
        m_viewPan = { 0.0, 0.0 };
        const auto radius = std::max(glm::length(maximum - minimum) * 0.5, 0.1);
        m_sceneScale = radius;
        const auto aspect = static_cast<double>(std::max(width, 1)) / std::max(height, 1);
        m_viewHalfHeight = radius * 1.1 / std::min(aspect, 1.0);
    }

    void pickRotationPivot(const int width, const int height, const int viewportLeft, const int viewportTop) {
        constexpr double PICK_RADIUS_PIXELS = 30.0;
        const auto aspect = static_cast<double>(width) / height;
        const glm::dvec2 mouse { m_mouseX - viewportLeft, m_mouseY - viewportTop };
        auto bestDistanceSquared = PICK_RADIUS_PIXELS * PICK_RADIUS_PIXELS;
        std::optional<glm::dvec3> bestPoint;

        const auto project = [&](const glm::dvec3 &point) {
            const auto transformed = m_modelOrientation * displayOffset(point - m_modelPivot);
            const auto ndcX = (transformed.x + m_viewPan.x) / (m_viewHalfHeight * aspect);
            const auto ndcY = (transformed.z + m_viewPan.y) / m_viewHalfHeight;
            return glm::dvec2((ndcX + 1.0) * width * 0.5, (1.0 - ndcY) * height * 0.5);
        };

        const auto segmentHit = [&](const glm::dvec3 &from, const glm::dvec3 &to) {
            const auto screenFrom = project(from);
            const auto screenTo = project(to);
            const auto screenSegment = screenTo - screenFrom;
            const auto lengthSquared = glm::dot(screenSegment, screenSegment);
            const auto t = lengthSquared > 0.0
                ? std::clamp(glm::dot(mouse - screenFrom, screenSegment) / lengthSquared, 0.0, 1.0)
                : 0.0;
            const auto difference = mouse - glm::mix(screenFrom, screenTo, t);
            return std::pair { glm::dot(difference, difference), t };
        };

        const auto considerSegment = [&](const glm::dvec3 &from, const glm::dvec3 &to) {
            const auto [distanceSquared, t] = segmentHit(from, to);

            if(distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestPoint = glm::mix(from, to, t);
            }
        };

        const auto considerTriadAxis = [&](const glm::dvec3 &origin, const glm::dvec3 &axisEnd) {
            const auto [distanceSquared, _] = segmentHit(origin, axisEnd);

            // Triads are drawn over the toolpath, so give them priority when
            // their projected geometry is equally close to the cursor.
            if(distanceSquared <= bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestPoint = origin;
            }
        };

        m_worker.lock([&] {
            m_worker.toolpath().foreachCommand([&](const ngc::MachineCommand &command) {
                std::visit([&](const auto &value) {
                    using T = std::decay_t<decltype(value)>;

                    if constexpr(std::same_as<T, ngc::MoveLine>) {
                        considerSegment(
                            { value.from().x, value.from().y, value.from().z },
                            { value.to().x, value.to().y, value.to().z });
                    } else if constexpr(std::same_as<T, ngc::ProbeMove>) {
                        considerSegment(
                            { value.from().x, value.from().y, value.from().z },
                            { value.target().x, value.target().y, value.target().z });
                    } else if constexpr(std::same_as<T, ngc::MoveArc>) {
                        interpolate(value, 60, [&](const glm::dvec3 &from, const glm::dvec3 &to, bool, bool) {
                            considerSegment(from, to);
                        });
                    }
                }, command);
            });

            const glm::dvec3 machineOrigin { 0.0, 0.0, 0.0 };
            considerTriadAxis(machineOrigin, machineOrigin + glm::dvec3(MACHINE_TRIAD_LENGTH, 0.0, 0.0));
            considerTriadAxis(machineOrigin, machineOrigin + glm::dvec3(0.0, MACHINE_TRIAD_LENGTH, 0.0));
            considerTriadAxis(machineOrigin, machineOrigin + glm::dvec3(0.0, 0.0, MACHINE_TRIAD_LENGTH));
            for(const auto &workCoordinateSystem : m_worker.toolpath().workCoordinateSystems()) {
                const auto &offset = workCoordinateSystem.offset;
                const glm::dvec3 workOrigin { offset.x, offset.y, offset.z };
                considerTriadAxis(workOrigin, workOrigin + glm::dvec3(WORK_TRIAD_LENGTH, 0.0, 0.0));
                considerTriadAxis(workOrigin, workOrigin + glm::dvec3(0.0, WORK_TRIAD_LENGTH, 0.0));
                considerTriadAxis(workOrigin, workOrigin + glm::dvec3(0.0, 0.0, WORK_TRIAD_LENGTH));
            }
        });

        if(bestPoint) {
            // Preserve the current model transform while changing the point
            // about which subsequent rotations are applied.
            const auto compensation = m_modelOrientation * displayOffset(*bestPoint - m_modelPivot);
            m_viewPan.x += compensation.x;
            m_viewPan.y += compensation.z;
            m_modelPivot = *bestPoint;
        }
    }

    void rebuildPreviewRenderCache() {
        if(m_worker.busy()) return;

        m_worker.lock([&] {
            const auto &toolpath = m_worker.toolpath();
            if(m_previewRenderCache.revision == toolpath.revision()) return;

            auto &cache = m_previewRenderCache;
            cache.feedLines.clear();
            cache.rapidLines.clear();
            cache.g53FeedLines.clear();
            cache.g53RapidLines.clear();
            cache.arcLines.clear();
            cache.probeLines.clear();
            cache.g64SplineLines.clear();
            cache.g64ClusterSplineLines.clear();
            cache.g64ControlPolygon.clear();
            cache.g64ControlPoints.clear();
            cache.darkPoints.clear();
            cache.lightPoints.clear();

            const auto point = [](const ngc::position_t &value) {
                return glm::dvec3(value.x, value.y, value.z);
            };
            const auto segment = [](auto &vertices, const glm::dvec3 &from, const glm::dvec3 &to) {
                vertices.push_back(from);
                vertices.push_back(to);
            };

            std::vector<ngc::experimental::JunctionEntity> splineEntities;
            std::optional<double> splineTolerance;
            const auto flushSpline = [&] {
                if(!splineTolerance) return;
                std::vector<bool> coveredJunctions(splineEntities.size() > 1 ? splineEntities.size() - 1 : 0, false);
                std::vector<bool> clusterEntity(splineEntities.size(), false);
                for(std::size_t i = 0; i < splineEntities.size(); ++i) {
                    clusterEntity[i] = splineEntities[i].length < 2.0 * *splineTolerance;
                }
                for(std::size_t entity = 0; entity < splineEntities.size();) {
                    if(!clusterEntity[entity]) { ++entity; continue; }
                    const auto firstShort = entity;
                    while(entity + 1 < splineEntities.size()
                          && clusterEntity[entity + 1]) ++entity;
                    const auto lastShort = entity;
                    if(firstShort > 0 && lastShort + 1 < splineEntities.size()) {
                        std::vector<ngc::experimental::JunctionEntity> clusterEntities(
                            splineEntities.begin() + static_cast<std::ptrdiff_t>(firstShort - 1),
                            splineEntities.begin() + static_cast<std::ptrdiff_t>(lastShort + 2));
                        if(const auto cluster = ngc::experimental::fitCluster(clusterEntities, *splineTolerance)) {
                            for(std::size_t junction = firstShort - 1; junction <= lastShort; ++junction) {
                                coveredJunctions[junction] = true;
                            }
                            for(const auto &span : cluster->spans) {
                                ngc::experimental::tessellateJunction(span, cache.g64ClusterSplineLines);
                                cache.g64ControlPoints.insert(cache.g64ControlPoints.end(),
                                    span.controlPoints.begin(), span.controlPoints.end());
                                for(std::size_t control = 1; control < span.controlPoints.size(); ++control) {
                                    segment(cache.g64ControlPolygon,
                                            span.controlPoints[control - 1], span.controlPoints[control]);
                                }
                            }
                        }
                    }
                    ++entity;
                }
                for(std::size_t i = 1; i < splineEntities.size(); ++i) {
                    if(coveredJunctions[i - 1]) continue;
                    const auto blend = ngc::experimental::fitJunction(
                        splineEntities[i - 1], splineEntities[i], *splineTolerance);
                    if(!blend) continue;
                    const auto &splineControlPoints = blend->controlPoints;
                    ngc::experimental::tessellateJunction(*blend, cache.g64SplineLines);
                    cache.g64ControlPoints.insert(cache.g64ControlPoints.end(),
                                                  splineControlPoints.begin(), splineControlPoints.end());
                    for(std::size_t i = 1; i < splineControlPoints.size(); ++i) {
                        segment(cache.g64ControlPolygon, splineControlPoints[i - 1], splineControlPoints[i]);
                    }
                }
                splineEntities.clear();
                splineTolerance.reset();
            };
            const auto beginSpline = [&](const std::optional<double> programmedTolerance) {
                constexpr double DEFAULT_EXPERIMENTAL_TOLERANCE = 0.001;
                const auto tolerance = std::max(programmedTolerance.value_or(DEFAULT_EXPERIMENTAL_TOLERANCE), 1e-9);
                if(splineTolerance && std::abs(*splineTolerance - tolerance) > 1e-12) flushSpline();
                splineTolerance = tolerance;
            };
            const auto lineEntity = [](const glm::dvec3 &from, const glm::dvec3 &to) {
                const auto delta = to - from;
                const auto length = glm::length(delta);
                const auto tangent = length > 1e-12 ? delta / length : glm::dvec3(1.0, 0.0, 0.0);
                return ngc::experimental::JunctionEntity {
                    .length = length,
                    .stateAtDistance = [=](const double distance) {
                        return ngc::experimental::JunctionState {
                            .position = from + tangent * std::clamp(distance, 0.0, length),
                            .tangent = tangent,
                            .curvature = {},
                        };
                    },
                };
            };
            const auto arcEntity = [&](const ngc::MoveArc &arc) {
                const auto length = ngc::simulation_detail::pathLength(arc);
                const auto positionAt = [arc, length](const double distance) {
                    const auto value = ngc::simulation_detail::interpolate(
                        arc, length > 1e-12 ? std::clamp(distance / length, 0.0, 1.0) : 0.0);
                    return glm::dvec3(value.x, value.y, value.z);
                };
                return ngc::experimental::JunctionEntity {
                    .length = length,
                    .stateAtDistance = [=](const double distance) {
                        const auto s = std::clamp(distance, 0.0, length);
                        const auto h = std::clamp(length * 1e-4, 1e-7, std::max(length * 0.01, 1e-7));
                        glm::dvec3 tangent;
                        glm::dvec3 curvature;
                        if(s <= h) {
                            const auto p0 = positionAt(s);
                            const auto p1 = positionAt(std::min(s + h, length));
                            const auto p2 = positionAt(std::min(s + 2.0 * h, length));
                            tangent = glm::normalize(p1 - p0);
                            curvature = (p2 - 2.0 * p1 + p0) / (h * h);
                        } else if(s + h >= length) {
                            const auto p0 = positionAt(s);
                            const auto p1 = positionAt(std::max(s - h, 0.0));
                            const auto p2 = positionAt(std::max(s - 2.0 * h, 0.0));
                            tangent = glm::normalize(p0 - p1);
                            curvature = (p0 - 2.0 * p1 + p2) / (h * h);
                        } else {
                            const auto before = positionAt(s - h);
                            const auto current = positionAt(s);
                            const auto after = positionAt(s + h);
                            tangent = glm::normalize(after - before);
                            curvature = (after - 2.0 * current + before) / (h * h);
                        }
                        return ngc::experimental::JunctionState {
                            .position = positionAt(s), .tangent = tangent, .curvature = curvature };
                    },
                };
            };

            const auto &commands = toolpath.commands();
            for(std::size_t commandIndex = 0; commandIndex < commands.size(); ++commandIndex) {
                const auto &command = commands[commandIndex];
                if(const auto line = std::get_if<ngc::MoveLine>(&command)) {
                    auto *vertices = &cache.feedLines;
                    const auto rapid = line->speed() == -1.0;
                    if(line->machineCoordinates()) vertices = rapid ? &cache.g53RapidLines : &cache.g53FeedLines;
                    else if(rapid) vertices = &cache.rapidLines;
                    const auto from = point(line->from());
                    const auto to = point(line->to());
                    segment(*vertices, from, to);
                    cache.darkPoints.push_back(from);
                    cache.darkPoints.push_back(to);
                    if(toolpath.g64Active(commandIndex) && !rapid && !line->machineCoordinates()) {
                        beginSpline(toolpath.g64Tolerance(commandIndex));
                        splineEntities.push_back(lineEntity(from, to));
                    } else {
                        flushSpline();
                    }
                } else if(const auto probe = std::get_if<ngc::ProbeMove>(&command)) {
                    flushSpline();
                    segment(cache.probeLines, point(probe->from()), point(probe->target()));
                } else if(const auto arc = std::get_if<ngc::MoveArc>(&command)) {
                    auto arcRenderResolution = 60;
                    if(toolpath.g64Active(commandIndex)) {
                        beginSpline(toolpath.g64Tolerance(commandIndex));
                        splineEntities.push_back(arcEntity(*arc));
                        const auto geometry = ngc::simulation_detail::arcGeometry(*arc);
                        if(geometry) {
                            const auto radius = std::sqrt(geometry->startArm.x * geometry->startArm.x
                                + geometry->startArm.y * geometry->startArm.y
                                + geometry->startArm.z * geometry->startArm.z);
                            const auto displayTolerance = std::max(*splineTolerance * 0.02, 1e-9);
                            if(radius > displayTolerance) {
                                const auto halfAngle = std::acos(std::clamp(
                                    1.0 - displayTolerance / radius, -1.0, 1.0));
                                if(halfAngle > 0.0) {
                                    arcRenderResolution = std::clamp(
                                        static_cast<int>(std::ceil(std::numbers::pi / halfAngle)), 60, 8192);
                                }
                            }
                        }
                    } else flushSpline();
                    interpolate(*arc, arcRenderResolution, [&](const glm::dvec3 &from, const glm::dvec3 &to,
                                              const bool startPoint, const bool endPoint) {
                        segment(cache.arcLines, from, to);
                        if(startPoint) cache.darkPoints.push_back(from);
                        if(endPoint) cache.lightPoints.push_back(to);
                    });
                } else {
                    flushSpline();
                }
            }
            flushSpline();
            cache.revision = toolpath.revision();
        });
    }

    void render3D() {
        rebuildPreviewRenderCache();
        int windowWidth, windowHeight, framebufferWidth, framebufferHeight;
        glfwGetWindowSize(m_window, &windowWidth, &windowHeight);
        glfwGetFramebufferSize(m_window, &framebufferWidth, &framebufferHeight);
        const auto viewportLeft = static_cast<int>(std::round(m_viewportRect.x));
        const auto viewportTop = static_cast<int>(std::round(m_viewportRect.y));
        const auto width = std::max(static_cast<int>(std::round(m_viewportRect.z - m_viewportRect.x)), 1);
        const auto height = std::max(static_cast<int>(std::round(m_viewportRect.w - m_viewportRect.y)), 1);
        const auto scaleX = static_cast<double>(framebufferWidth) / std::max(windowWidth, 1);
        const auto scaleY = static_cast<double>(framebufferHeight) / std::max(windowHeight, 1);
        glViewport(static_cast<int>(std::round(viewportLeft * scaleX)),
                   framebufferHeight - static_cast<int>(std::round((viewportTop + height) * scaleY)),
                   std::max(static_cast<int>(std::round(width * scaleX)), 1),
                   std::max(static_cast<int>(std::round(height * scaleY)), 1));

        const auto navigationActive = m_mouseX >= m_viewportRect.x && m_mouseX <= m_viewportRect.z
            && m_mouseY >= m_viewportRect.y && m_mouseY <= m_viewportRect.w
            && !ImGui::GetIO().WantCaptureMouse;
        const auto middleDown = ImGui::IsMouseDown(ImGuiMouseButton_Middle);
        const auto ctrl = ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl);
        const auto shift = ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift);

        if(navigationActive && ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            fitToolpathToView(width, height);
        }

        if(navigationActive && ImGui::IsMouseClicked(ImGuiMouseButton_Middle) && !ctrl && !shift) {
            pickRotationPivot(width, height, viewportLeft, viewportTop);
        }

        if(navigationActive && middleDown && !ctrl && !shift) {
            const auto horizontal = glm::angleAxis(-m_mouseDX * 0.01, glm::dvec3(0.0, 0.0, 1.0));
            const auto vertical = glm::angleAxis(-m_mouseDY * 0.01, glm::dvec3(1.0, 0.0, 0.0));
            m_modelOrientation = glm::normalize(vertical * horizontal * m_modelOrientation);
        }

        if(navigationActive && middleDown && ctrl) {
            const auto worldUnitsPerPixel = 2.0 * m_viewHalfHeight / height;
            m_viewPan.x += m_mouseDX * worldUnitsPerPixel;
            m_viewPan.y -= m_mouseDY * worldUnitsPerPixel;
        }

        const auto scrollDelta = std::exchange(m_scrollDelta, 0.0);
        auto scaleFactor = 1.0;
        if(navigationActive && middleDown && shift) {
            scaleFactor *= std::exp(-m_mouseDY * 0.01);
        }
        if(navigationActive && scrollDelta != 0.0) {
            scaleFactor *= std::exp(scrollDelta * 0.15);
        }

        const auto aspect = static_cast<double>(width) / height;
        if(scaleFactor != 1.0) {
            const auto oldHalfHeight = m_viewHalfHeight;
            const auto newHalfHeight = std::clamp(oldHalfHeight * scaleFactor, m_sceneScale * 1e-9, m_sceneScale * 1e9);
            const auto mouseNdcX = 2.0 * (m_mouseX - viewportLeft) / width - 1.0;
            const auto mouseNdcY = 1.0 - 2.0 * (m_mouseY - viewportTop) / height;

            // Changing an orthographic projection normally zooms around the
            // screen center. Shift the view so the point under the cursor
            // remains under the cursor as the projection scale changes.
            m_viewPan.x += mouseNdcX * aspect * (newHalfHeight - oldHalfHeight);
            m_viewPan.y += mouseNdcY * (newHalfHeight - oldHalfHeight);
            m_viewHalfHeight = newHalfHeight;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-m_viewHalfHeight * aspect, m_viewHalfHeight * aspect,
                -m_viewHalfHeight, m_viewHalfHeight, -m_sceneScale * 1e6, m_sceneScale * 1e6);

        const auto view = glm::lookAt(glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(0.0), glm::dvec3(0.0, 0.0, 1.0));
        auto model = glm::translate(glm::dmat4(1.0), glm::dvec3(m_viewPan.x, 0.0, m_viewPan.y));
        model *= glm::mat4_cast(m_modelOrientation);
        model = glm::scale(model, glm::dvec3(1.0, -1.0, 1.0));
        model = glm::translate(model, -m_modelPivot);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixd(glm::value_ptr(view * model));

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glEnable(GL_POINT_SMOOTH);
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

        struct CoordinateDisplay {
            ngc::WorkCoordinateSystem active;
            std::vector<ngc::WorkCoordinateSystem> used;
        };

        auto coordinates = m_worker.lock([&] {
            const auto &machine = m_worker.machine();
            return CoordinateDisplay {
                .active = {
                    .name = std::string(ngc::name(*machine.state().modeCoordSys)),
                    .offset = machine.workOffset(),
                },
                .used = m_worker.toolpath().workCoordinateSystems(),
            };
        });
        const auto simulationCoordinates = m_simulation.snapshot();
        for(const auto &workCoordinateSystem : simulationCoordinates.usedWorkCoordinateSystems) {
            const auto match = std::ranges::find_if(coordinates.used, [&](const auto &value) {
                return value.name == workCoordinateSystem.name;
            });
            if(match == coordinates.used.end()) coordinates.used.push_back(workCoordinateSystem);
            else *match = workCoordinateSystem;
        }
        const auto simulationActive = simulationCoordinates.status == ngc::SimulationStatus::Running
            || simulationCoordinates.status == ngc::SimulationStatus::Paused;
        if(simulationActive && simulationCoordinates.activeWorkCoordinateSystem) {
            coordinates.active = *simulationCoordinates.activeWorkCoordinateSystem;
        }

        const auto drawTriad = [](const glm::dvec3 &origin, const double length, const bool dashed, const float intensity) {
            if(dashed) {
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(1, 0x3333);
            }

            glBegin(GL_LINES);
            glColor3f(intensity, 0.2f * intensity, 0.2f * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x + length, origin.y, origin.z);
            glColor3f(0.2f * intensity, intensity, 0.2f * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x, origin.y + length, origin.z);
            glColor3f(0.25f * intensity, 0.45f * intensity, intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x, origin.y, origin.z + length);
            glEnd();

            if(dashed) {
                glDisable(GL_LINE_STIPPLE);
            }

            glEnable(GL_POINT_SMOOTH);
            glPointSize(dashed ? 7.0f : 9.0f);
            glBegin(GL_POINTS);
            glColor3f(intensity, (dashed ? 0.75f : 0.95f) * intensity,
                      (dashed ? 0.15f : 0.95f) * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glEnd();
        };

        glLineWidth(2.0f);
        drawTriad({ 0.0, 0.0, 0.0 }, MACHINE_TRIAD_LENGTH, false, 0.55f);
        for(const auto &workCoordinateSystem : coordinates.used) {
            if(workCoordinateSystem.name == coordinates.active.name) continue;
            const auto &offset = workCoordinateSystem.offset;
            glLineWidth(2.0f);
            drawTriad({ offset.x, offset.y, offset.z }, WORK_TRIAD_LENGTH, true, 0.35f);
        }
        glLineWidth(5.0f);
        drawTriad({ coordinates.active.offset.x, coordinates.active.offset.y, coordinates.active.offset.z },
                  WORK_TRIAD_LENGTH, true, 1.0f);
        glLineWidth(1.0f);
        glPointSize(1.0f);

        const auto projectPoint = [&](const glm::dvec3 &point) {
            const auto transformed = m_modelOrientation * displayOffset(point - m_modelPivot);
            const auto ndcX = (transformed.x + m_viewPan.x) / (m_viewHalfHeight * aspect);
            const auto ndcY = (transformed.z + m_viewPan.y) / m_viewHalfHeight;
            return ImVec2(
                static_cast<float>(viewportLeft + (ndcX + 1.0) * width * 0.5),
                static_cast<float>(viewportTop + (1.0 - ndcY) * height * 0.5));
        };

        const auto machineOrigin = projectPoint({ 0.0, 0.0, 0.0 });
        const auto workOrigin = projectPoint({ coordinates.active.offset.x, coordinates.active.offset.y, coordinates.active.offset.z });
        const auto screenDifference = ImVec2(workOrigin.x - machineOrigin.x, workOrigin.y - machineOrigin.y);
        const auto originsOverlap = screenDifference.x * screenDifference.x + screenDifference.y * screenDifference.y < 24.0f * 24.0f;
        const auto machineLabelPosition = ImVec2(machineOrigin.x + 8.0f, machineOrigin.y - (originsOverlap ? 28.0f : 18.0f));
        const auto workLabelPosition = ImVec2(workOrigin.x + 8.0f, workOrigin.y + (originsOverlap ? 6.0f : -18.0f));

        auto *drawList = ImGui::GetForegroundDrawList(ImGui::GetMainViewport());
        drawList->AddText(machineLabelPosition, IM_COL32(180, 180, 180, 255), "MCS");
        drawList->AddText(
            workLabelPosition,
            IM_COL32(255, 205, 45, 255),
            std::format("{} WCS", coordinates.active.name).c_str());
        for(const auto &workCoordinateSystem : coordinates.used) {
            if(workCoordinateSystem.name == coordinates.active.name) continue;
            const auto &offset = workCoordinateSystem.offset;
            const auto labelPosition = projectPoint({ offset.x, offset.y, offset.z });
            drawList->AddText({ labelPosition.x + 8.0f, labelPosition.y - 18.0f },
                              IM_COL32(170, 145, 70, 210),
                              std::format("{} WCS", workCoordinateSystem.name).c_str());
        }

        const auto fpsText = std::format("FPS: {:.1f}", ImGui::GetIO().Framerate);
        const auto fpsSize = ImGui::CalcTextSize(fpsText.c_str());
        constexpr float FPS_PADDING = 8.0f;
        const ImVec2 fpsPosition {
            m_viewportRect.z - fpsSize.x - FPS_PADDING,
            m_viewportRect.w - fpsSize.y - FPS_PADDING,
        };
        drawList->AddText({ fpsPosition.x + 1.0f, fpsPosition.y + 1.0f },
                          IM_COL32(0, 0, 0, 210), fpsText.c_str());
        drawList->AddText(fpsPosition, IM_COL32(210, 220, 225, 255), fpsText.c_str());

        const auto drawVertices = [](const std::vector<glm::dvec3> &vertices, const GLenum primitive,
                                     const float red, const float green, const float blue,
                                     const int stippleFactor = 0, const GLushort stipplePattern = 0xFFFF) {
            if(vertices.empty()) return;
            if(stippleFactor > 0) {
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(stippleFactor, stipplePattern);
            }
            glColor3f(red, green, blue);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_DOUBLE, sizeof(glm::dvec3), &vertices.front().x);
            glDrawArrays(primitive, 0, static_cast<GLsizei>(vertices.size()));
            glDisableClientState(GL_VERTEX_ARRAY);
            if(stippleFactor > 0) glDisable(GL_LINE_STIPPLE);
        };

        glLineWidth(2.0f);
        drawVertices(m_previewRenderCache.feedLines, GL_LINES, 0.4f, 0.4f, 1.0f);
        drawVertices(m_previewRenderCache.g53FeedLines, GL_LINES, 1.0f, 1.0f, 0.4f);
        drawVertices(m_previewRenderCache.arcLines, GL_LINES, 0.4f, 1.0f, 0.4f);
        drawVertices(m_previewRenderCache.probeLines, GL_LINES, 1.0f, 0.4f, 0.2f);
        glLineWidth(1.0f);
        drawVertices(m_previewRenderCache.g64SplineLines, GL_LINES, 1.0f, 0.2f, 1.0f);
        drawVertices(m_previewRenderCache.g64ClusterSplineLines, GL_LINES, 0.1f, 1.0f, 1.0f);
        glLineWidth(1.0f);
        drawVertices(m_previewRenderCache.g64ControlPolygon, GL_LINES, 0.95f, 0.65f, 1.0f, 2, 0x3333);
        glPointSize(6.0f);
        drawVertices(m_previewRenderCache.g64ControlPoints, GL_POINTS, 1.0f, 0.75f, 1.0f);
        glPointSize(3.0f);
        glLineWidth(2.0f);
        drawVertices(m_previewRenderCache.g53RapidLines, GL_LINES, 1.0f, 1.0f, 0.4f, 10, 0x5555);
        drawVertices(m_previewRenderCache.rapidLines, GL_LINES, 0.4f, 0.4f, 1.0f, 4, 0x5555);
        glEnable(GL_POINT_SMOOTH);
        glPointSize(3.0f);
        drawVertices(m_previewRenderCache.darkPoints, GL_POINTS, 0.0f, 0.0f, 0.0f);
        drawVertices(m_previewRenderCache.lightPoints, GL_POINTS, 0.1f, 0.55f, 0.1f);
        glLineWidth(1.0f);

        const auto simulation = m_simulation.snapshot();
        if(simulation.status != ngc::SimulationStatus::Stopped) {
            drawToolWireframe(simulation.toolPose);
        }
    }

    void interpolate(const ngc::MoveArc &arc, int circleResolution, const std::function<void(const glm::dvec3 &start, const glm::dvec3 &end, const bool startPoint, const bool endPoint)> &callback) {
        const auto geometry = ngc::simulation_detail::arcGeometry(arc);
        const auto sweep = geometry ? geometry->sweep : 0.0;
        const auto segments = geometry && circleResolution > 0
            ? std::max(1, static_cast<int>(std::ceil(sweep * circleResolution / (2.0 * std::numbers::pi))))
            : 1;

        auto previous = arc.from();
        for(int i = 1; i <= segments; i++) {
            const auto current = ngc::simulation_detail::interpolate(arc, static_cast<double>(i) / segments);
            callback(
                { previous.x, previous.y, previous.z },
                { current.x, current.y, current.z },
                i == 1,
                i == segments);
            previous = current;
        }
    }

    void renderErrorWindow() {
        ImGui::OpenPopup("Error");

        if(ImGui::BeginPopupModal("Error", &m_enableErrorWindow, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
            ImGui::TextUnformatted(m_errorMessage.c_str());
            ImGui::PopStyleColor();
        }

        ImGui::EndPopup();
    }

    void saveMainProgram() {
        if(m_programSource.empty()) return;
        const auto &[source, name] = m_programSource.back();
        if(const auto result = ngc::writeFile(name, source); !result) {
            m_errorMessage = result.error().what();
            return;
        }
        m_programDirty = false;
        m_errorMessage.clear();
    }

    void rebuildMainProgramLines() {
        m_mainProgramLines.clear();
        if(m_programSource.empty()) return;

        std::istringstream stream(std::get<0>(m_programSource.back()));
        std::string line;
        while(std::getline(stream, line)) {
            if(!line.empty() && line.back() == '\r') line.pop_back();
            m_mainProgramLines.emplace_back(std::move(line));
        }
    }

    void renderToolbar(const ImGuiViewport &viewport, const ngc::SimulationSnapshot &simulation) {
        ImGui::SetNextWindowPos(viewport.Pos);
        ImGui::SetNextWindowSize({ viewport.Size.x, m_toolbarHeight });
        constexpr auto flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("##toolbar", nullptr, flags);

        if(ImGui::Button("Open G-code")) m_enableOpenDialog = true;
        ImGui::SameLine();

        const auto compiledMode = m_programPaneMode == ProgramPaneMode::Compiled;
        const auto canPreview = compiledMode && !m_programSource.empty() && !m_worker.busy() && !m_previewAfterCompile;
        ImGui::BeginDisabled(!canPreview);
        if(ImGui::Button("Preview")) {
            if(m_worker.compile(m_programSource)) m_previewAfterCompile = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(m_worker.busy());
        if(ImGui::Button("Clear Preview")) m_worker.clearToolpath();
        ImGui::EndDisabled();

        const auto simulationActive = simulation.status == ngc::SimulationStatus::Running
            || simulation.status == ngc::SimulationStatus::Paused;
        ImGui::SameLine();
        ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
        ImGui::SameLine();
        ImGui::BeginDisabled(!compiledMode || m_programSource.empty() || simulationActive);
        if(ImGui::Button("Simulate")) {
            m_simulation.setPlaybackRate(m_simulationRate);
            m_simulation.setRapidSpeed(m_simulatedRapidSpeed);
            m_simulation.start(m_programSource, m_tools, true);
            m_programPaneMode = ProgramPaneMode::Compiled;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!simulationActive);
        if(ImGui::Button(simulation.status == ngc::SimulationStatus::Paused ? "Resume" : "Pause")) {
            if(simulation.status == ngc::SimulationStatus::Paused) m_simulation.resume();
            else m_simulation.pause();
        }
        ImGui::SameLine();
        if(ImGui::Button("Stop")) m_simulation.stop();
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::BeginDisabled(simulationActive);
        if(ImGui::Button("Reset Simulation")) m_simulation.resetSimulation();
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        if(ImGui::InputDouble("Speed", &m_simulationRate, 0.1, 1.0, "%.2fx")) {
            m_simulationRate = std::clamp(m_simulationRate, 0.01, 1000.0);
            m_simulation.setPlaybackRate(m_simulationRate);
        }
        ImGui::SameLine();
        if(ImGui::Button("Parameters")) m_enableMemoryWindow = true;
        ImGui::SameLine();
        if(ImGui::Button("Tool Table")) {
            initToolTableStrings();
            m_enableToolWindow = true;
        }
        ImGui::End();
    }

    void renderProgramTab(const ngc::SimulationSnapshot &simulation) {
        const auto editMode = m_programPaneMode == ProgramPaneMode::Edit;
        ImGui::BeginDisabled(editMode);
        if(ImGui::Button("Edit")) m_programPaneMode = ProgramPaneMode::Edit;
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!editMode || m_programSource.empty() || m_worker.busy());
        if(ImGui::Button("Compile")) {
            m_programPaneMode = ProgramPaneMode::Compiled;
            if(!m_programSource.empty() && !m_worker.busy()) m_worker.compile(m_programSource);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_programDirty);
        if(ImGui::Button("Save")) saveMainProgram();
        ImGui::EndDisabled();

        if(m_programSource.empty()) {
            ImGui::TextDisabled("Open a G-code program to begin.");
            return;
        }

        auto &[source, name] = m_programSource.back();
        ImGui::SameLine();
        ImGui::TextDisabled("%s%s", std::filesystem::path(name).filename().string().c_str(), m_programDirty ? " *" : "");
        ImGui::Separator();

        ImGui::BeginChild("##program_content", { 0.0f, 0.0f }, ImGuiChildFlags_None);

        if(m_programPaneMode == ProgramPaneMode::Edit) {
            if(ImGui::InputTextMultiline("##gcode_editor", &source, { -1.0f, -1.0f },
                                         ImGuiInputTextFlags_AllowTabInput)) {
                m_programDirty = true;
                rebuildMainProgramLines();
            }
        } else {
            std::unordered_set<int> activeLines;
            std::optional<int> currentLine;
            for(const auto &block : simulation.activeBlocks) {
                if(block.source == name) activeLines.insert(block.line);
            }
            if(!simulation.activeBlocks.empty() && simulation.activeBlocks.back().source == name) {
                currentLine = simulation.activeBlocks.back().line;
            }
            const auto completedSource = simulation.completedLineFlags.find(name);

            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(m_mainProgramLines.size()));
            if(currentLine && *currentLine > 0 && *currentLine <= static_cast<int>(m_mainProgramLines.size())) {
                clipper.IncludeItemByIndex(*currentLine - 1);
            }
            while(clipper.Step()) {
                for(int index = clipper.DisplayStart; index < clipper.DisplayEnd; ++index) {
                    const auto lineNumber = index + 1;
                    const auto active = activeLines.contains(lineNumber);
                    const auto completed = completedSource != simulation.completedLineFlags.end()
                        && lineNumber < static_cast<int>(completedSource->second.size())
                        && completedSource->second[lineNumber] != 0;
                    const auto current = currentLine == lineNumber;
                    const auto highlight = current
                        ? ImVec4(0.75f, 0.55f, 0.08f, 0.75f)
                        : active ? ImVec4(0.65f, 0.38f, 0.08f, 0.55f)
                        : completed ? ImVec4(0.15f, 0.45f, 0.22f, 0.55f) : ImVec4(0, 0, 0, 0);
                    ImGui::PushStyleColor(ImGuiCol_Header, highlight);
                    ImGui::Selectable(std::format("{:5}  {}", lineNumber, m_mainProgramLines[index]).c_str(),
                                      active || completed);
                    if(current) ImGui::SetScrollHereY(0.5f);
                    ImGui::PopStyleColor();
                }
            }
        }
        ImGui::EndChild();
    }

    void submitMdiBlock() {
        if(m_mdiInput.find_first_not_of(" \t\r\n") == std::string::npos) return;

        m_mdiHistory.emplace_back(std::move(m_mdiInput));
        std::string source(m_mdiHistory.size() - 1, '\n');
        source += m_mdiHistory.back();
        source += '\n';

        auto programs = m_autoloadSource;
        programs.emplace_back(std::move(source), std::string(MDI_SOURCE_NAME));

        m_simulation.setPlaybackRate(m_simulationRate);
        m_simulation.setRapidSpeed(m_simulatedRapidSpeed);
        if(!m_simulation.start(programs, m_tools, true)) {
            m_mdiInput = std::move(m_mdiHistory.back());
            m_mdiHistory.pop_back();
            m_errorMessage = "MDI simulation cannot start while another simulation is active";
            return;
        }

        m_mdiInput.clear();
        m_errorMessage.clear();
    }

    void renderMdiTab(const ngc::SimulationSnapshot &simulation) {
        std::unordered_set<int> activeLines;
        std::optional<int> currentLine;
        for(const auto &block : simulation.activeBlocks) {
            if(block.source == MDI_SOURCE_NAME) activeLines.insert(block.line);
        }
        if(!simulation.activeBlocks.empty() && simulation.activeBlocks.back().source == MDI_SOURCE_NAME) {
            currentLine = simulation.activeBlocks.back().line;
        }
        const auto completedSource = simulation.completedLineFlags.find(std::string(MDI_SOURCE_NAME));

        const auto inputHeight = ImGui::GetFrameHeightWithSpacing();
        if(ImGui::BeginChild("##mdi_history", { 0.0f, -inputHeight }, ImGuiChildFlags_Border)) {
            ImGuiListClipper clipper;
            clipper.Begin(static_cast<int>(m_mdiHistory.size()));
            while(clipper.Step()) {
                for(int index = clipper.DisplayStart; index < clipper.DisplayEnd; ++index) {
                    const auto lineNumber = index + 1;
                    const auto active = activeLines.contains(lineNumber);
                    const auto completed = completedSource != simulation.completedLineFlags.end()
                        && lineNumber < static_cast<int>(completedSource->second.size())
                        && completedSource->second[lineNumber] != 0;
                    const auto current = currentLine == lineNumber;
                    const auto highlight = current
                        ? ImVec4(0.75f, 0.55f, 0.08f, 0.75f)
                        : active ? ImVec4(0.65f, 0.38f, 0.08f, 0.55f)
                        : completed ? ImVec4(0.15f, 0.45f, 0.22f, 0.55f) : ImVec4(0, 0, 0, 0);
                    const auto label = std::format("{:5}  {}", index + 1, m_mdiHistory[index]);
                    ImGui::PushStyleColor(ImGuiCol_Header, highlight);
                    if(ImGui::Selectable(label.c_str())) m_mdiInput = m_mdiHistory[index];
                    if(current) ImGui::SetScrollHereY(0.5f);
                    ImGui::PopStyleColor();
                }
            }
        }
        ImGui::EndChild();

        const auto buttonWidth = ImGui::CalcTextSize("Run").x + ImGui::GetStyle().FramePadding.x * 2.0f;
        ImGui::SetNextItemWidth(std::max(1.0f, ImGui::GetContentRegionAvail().x - buttonWidth
                                               - ImGui::GetStyle().ItemSpacing.x));
        const auto simulationActive = simulation.status == ngc::SimulationStatus::Running
            || simulation.status == ngc::SimulationStatus::Paused;
        ImGui::BeginDisabled(simulationActive);
        const auto submittedWithEnter = ImGui::InputText("##mdi_input", &m_mdiInput,
                                                         ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        const auto submittedWithButton = ImGui::Button("Run");
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Run this block on the simulated machine.");
        ImGui::EndDisabled();
        if(submittedWithEnter || submittedWithButton) submitMdiBlock();
    }

    void renderProgramPane(const ImGuiViewport &viewport, const ngc::SimulationSnapshot &simulation,
                           const float contentBottom) {
        ImGui::SetNextWindowPos({ viewport.Pos.x, viewport.Pos.y + m_toolbarHeight });
        ImGui::SetNextWindowSize({ m_programPaneWidth, contentBottom - viewport.Pos.y - m_toolbarHeight });
        constexpr auto flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("##program_pane", nullptr, flags);

        if(ImGui::BeginTabBar("##left_pane_tabs")) {
            if(ImGui::BeginTabItem("Program")) {
                renderProgramTab(simulation);
                ImGui::EndTabItem();
            }
            if(ImGui::BeginTabItem("MDI")) {
                renderMdiTab(simulation);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }

    void renderStatusBar(const ImGuiViewport &viewport, const ngc::SimulationSnapshot &simulation) {
        const auto y = viewport.Pos.y + viewport.Size.y - m_statusBarHeight;
        ImGui::SetNextWindowPos({ viewport.Pos.x, y });
        ImGui::SetNextWindowSize({ viewport.Size.x, m_statusBarHeight });
        constexpr auto flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("##status_bar", nullptr, flags);

        const auto statusText = [&] {
            switch(simulation.status) {
                case ngc::SimulationStatus::Stopped: return "Stopped";
                case ngc::SimulationStatus::Running: return "Running";
                case ngc::SimulationStatus::Paused: return "Paused";
                case ngc::SimulationStatus::Completed: return "Completed";
                case ngc::SimulationStatus::Error: return "Error";
            }
            return "Unknown";
        }();
        ImGui::Text("Simulation: %s    Tool XYZ: %.4f, %.4f, %.4f",
                    statusText, simulation.toolPosition.x, simulation.toolPosition.y, simulation.toolPosition.z);
        auto modalGCodes = m_worker.lock([&] { return m_worker.machine().activeModalGCodes(); });
        const auto simulationHasModalState = simulation.status != ngc::SimulationStatus::Stopped
            && !simulation.activeModalGCodes.empty();
        if(simulationHasModalState) modalGCodes = simulation.activeModalGCodes;
        std::string modalText;
        for(const auto &code : modalGCodes) {
            if(!modalText.empty()) modalText += ' ';
            modalText += code;
        }
        if(!modalText.empty()) {
            const auto textWidth = ImGui::CalcTextSize(modalText.c_str()).x;
            const auto rightAlignedX = ImGui::GetWindowContentRegionMax().x - textWidth;
            ImGui::SameLine(std::max(ImGui::GetCursorPosX(), rightAlignedX));
            ImGui::TextDisabled("%s", modalText.c_str());
        }
        ImGui::Separator();
        if(!m_errorMessage.empty()) ImGui::TextColored(ImVec4_Redish, "ERROR: %s", m_errorMessage.c_str());
        for(const auto &error : m_worker.parserErrors()) {
            ImGui::TextColored(ImVec4_Redish, "ERROR: %s", error.text().c_str());
        }
        const auto renderInterpreterMessages = [](const auto &messages) {
            for(const auto &message : messages) {
                if(message.kind == ngc::InterpreterStatusKind::Error) {
                    ImGui::TextColored(ImVec4_Redish, "ERROR: %s", message.text.c_str());
                } else {
                    ImGui::TextColored(ImVec4_Blueish, "PRINT: %s", message.text.c_str());
                }
            }
        };
        renderInterpreterMessages(m_worker.statusMessages());
        renderInterpreterMessages(simulation.statusMessages);
        ImGui::End();
    }

    void renderSplitters(const ImGuiViewport &viewport, const float contentBottom) {
        auto *drawList = ImGui::GetForegroundDrawList();
        const auto splitterColor = IM_COL32(95, 105, 115, 255);
        drawList->AddRectFilled({ viewport.Pos.x + m_programPaneWidth - 2.0f, viewport.Pos.y + m_toolbarHeight },
                                { viewport.Pos.x + m_programPaneWidth + 2.0f, contentBottom }, splitterColor);
        ImGui::SetNextWindowPos({ viewport.Pos.x + m_programPaneWidth - 4.0f, viewport.Pos.y + m_toolbarHeight });
        ImGui::SetNextWindowSize({ 8.0f, contentBottom - viewport.Pos.y - m_toolbarHeight });
        ImGui::Begin("##program_splitter", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground
            | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::InvisibleButton("##drag_program", { -FLT_MIN, -FLT_MIN });
        if(ImGui::IsItemActive()) {
            m_programPaneWidth = std::clamp(m_programPaneWidth + ImGui::GetIO().MouseDelta.x, 260.0f,
                                            viewport.Size.x - 240.0f);
        }
        ImGui::End();

        drawList->AddRectFilled({ viewport.Pos.x, contentBottom - 2.0f },
                                { viewport.Pos.x + viewport.Size.x, contentBottom + 2.0f }, splitterColor);
        ImGui::SetNextWindowPos({ viewport.Pos.x, contentBottom - 4.0f });
        ImGui::SetNextWindowSize({ viewport.Size.x, 8.0f });
        ImGui::Begin("##status_splitter", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground
            | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::InvisibleButton("##drag_status", { -FLT_MIN, -FLT_MIN });
        if(ImGui::IsItemActive()) {
            m_statusBarHeight = std::clamp(m_statusBarHeight - ImGui::GetIO().MouseDelta.y, 80.0f,
                                           viewport.Size.y - m_toolbarHeight - 120.0f);
        }
        ImGui::End();
    }

    void render() {
        if(m_previewAfterCompile && !m_worker.busy()) {
            if(m_worker.compiled()) m_worker.execute();
            m_previewAfterCompile = false;
        }

        const auto simulation = m_simulation.snapshot();
        const auto &viewport = *ImGui::GetMainViewport();
        m_toolbarHeight = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
        const auto contentBottom = viewport.Pos.y + viewport.Size.y - m_statusBarHeight;
        m_viewportRect = {
            viewport.Pos.x + m_programPaneWidth,
            viewport.Pos.y + m_toolbarHeight,
            viewport.Pos.x + viewport.Size.x,
            contentBottom,
        };

        renderToolbar(viewport, simulation);
        renderProgramPane(viewport, simulation, contentBottom);
        renderStatusBar(viewport, simulation);
        renderSplitters(viewport, contentBottom);

        if(m_enableOpenDialog) renderOpenDialog();
        if(m_enableMemoryWindow) renderMemoryWindow();
        if(m_enableToolWindow) renderToolWindow();
    }

    void renderToolWindow() {
        ImGui::SetNextWindowSize({ 950.0f, 360.0f }, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSizeConstraints({ 500.0f, 220.0f }, { FLT_MAX, FLT_MAX });

        if(ImGui::Begin("Tool Table", &m_enableToolWindow)) {
            ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, { 0, 0 });
            const auto tableHeight = -ImGui::GetFrameHeightWithSpacing();

            if(ImGui::BeginTable("##tool_table", 9,
                                 ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                 ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable |
                                 ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY,
                                 { 0.0f, tableHeight })) {
                ImGui::TableSetupColumn("Tool Number");
                ImGui::TableSetupColumn("X");
                ImGui::TableSetupColumn("Y");
                ImGui::TableSetupColumn("Z");
                ImGui::TableSetupColumn("A");
                ImGui::TableSetupColumn("B");
                ImGui::TableSetupColumn("C");
                ImGui::TableSetupColumn("Diameter");
                ImGui::TableSetupColumn("Comment");
                ImGui::TableHeadersRow();

                for(std::size_t row = 0; row < m_toolStrings.size(); row++) {
                    auto &tool = m_toolStrings[row];
                    ImGui::PushID(static_cast<int>(row));
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##number", &tool.number);

                    ImGui::TableSetColumnIndex(1);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##x", &tool.x);

                    ImGui::TableSetColumnIndex(2);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##y", &tool.y);

                    ImGui::TableSetColumnIndex(3);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##z", &tool.z);

                    ImGui::TableSetColumnIndex(4);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##a", &tool.a);

                    ImGui::TableSetColumnIndex(5);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##b", &tool.b);

                    ImGui::TableSetColumnIndex(6);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##c", &tool.c);

                    ImGui::TableSetColumnIndex(7);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##diameter", &tool.diameter);

                    ImGui::TableSetColumnIndex(8);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##comment", &tool.comment);
                    ImGui::PopID();
                }

                ImGui::EndTable();
            }

            ImGui::PopStyleVar();

            if(ImGui::Button("Reload")) {
                reloadToolTable();
            }

            ImGui::SameLine();

            if(ImGui::Button("Save")) {
                saveToolTableStrings();
            }

            ImGui::SameLine();

            if(ImGui::Button("New")) {
                int nextToolNumber = 1;

                for(const auto &tool : m_toolStrings) {
                    if(const auto number = ngc::fromChars(tool.number)) {
                        nextToolNumber = std::max(nextToolNumber, static_cast<int>(*number) + 1);
                    }
                }

                m_toolStrings.emplace_back(tool_table_strings_t {
                    .number = ngc::toChars(nextToolNumber),
                    .x = "0",
                    .y = "0",
                    .z = "0",
                    .a = "0",
                    .b = "0",
                    .c = "0",
                    .diameter = "0",
                    .comment = {},
                });
            }
        }

        ImGui::End();
    }

    void renderMemoryWindow() {
        if(ImGui::Begin("System Parameters", &m_enableMemoryWindow)) {
            if(ImGui::BeginTable("", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("Address");
                ImGui::TableSetupColumn("Internal Name");
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Value");
                ImGui::TableHeadersRow();

                uint32_t address = 0;

                for(const auto &[var, name, addr, _flags, _value] : ngc::gVars) {
                    if(addr != 0) {
                        address = addr;
                    }

                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(std::format("{}", address).c_str());

                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(std::format("{}", ngc::name(var)).c_str());

                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextUnformatted(std::format("{}", name).c_str());

                    ImGui::TableSetColumnIndex(3);
                    auto value = m_worker.read(var);
                    ImGui::TextUnformatted(std::format("{}", value).c_str());

                    address++;
                }
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

    void renderProgramWindow() {
        if(ImGui::Begin("Program", &m_enableProgramWindow)) {
            if(ImGui::BeginChild("top", { 0, 0 }, ImGuiChildFlags_Border | ImGuiChildFlags_ResizeY)) {
                if(ImGui::Button("Compile Programs", {0,0}, m_worker.busy())) {
                    m_worker.compile(m_programSource);
                }

                if(m_worker.compiled()) {
                    if(ImGui::Button("Build Preview")) {
                        m_worker.execute();
                    }
                }

                const auto simulation = m_simulation.snapshot();
                const auto simulationActive = simulation.status == ngc::SimulationStatus::Running
                    || simulation.status == ngc::SimulationStatus::Paused;

                ImGui::SameLine();
                ImGui::BeginDisabled(m_programSource.empty() || simulationActive);
                if(ImGui::Button("Run Simulation")) {
                    m_simulation.setPlaybackRate(m_simulationRate);
                    m_simulation.setRapidSpeed(m_simulatedRapidSpeed);
                    m_simulation.start(m_programSource, m_tools, true);
                }
                ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::BeginDisabled(!simulationActive);
                if(simulation.status == ngc::SimulationStatus::Paused) {
                    if(ImGui::Button("Resume")) m_simulation.resume();
                } else {
                    if(ImGui::Button("Pause")) m_simulation.pause();
                }
                ImGui::SameLine();
                if(ImGui::Button("Stop")) m_simulation.stop();
                ImGui::EndDisabled();

                ImGui::SetNextItemWidth(120.0f);
                if(ImGui::InputDouble("Playback rate", &m_simulationRate, 0.1, 1.0, "%.2fx")) {
                    m_simulationRate = std::clamp(m_simulationRate, 0.01, 1000.0);
                    m_simulation.setPlaybackRate(m_simulationRate);
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120.0f);
                if(ImGui::InputDouble("Rapid speed", &m_simulatedRapidSpeed, 10.0, 100.0, "%.2f")) {
                    m_simulatedRapidSpeed = std::max(m_simulatedRapidSpeed, 1e-6);
                    m_simulation.setRapidSpeed(m_simulatedRapidSpeed);
                }

                const auto statusText = [&] {
                    switch(simulation.status) {
                        case ngc::SimulationStatus::Stopped: return "Stopped";
                        case ngc::SimulationStatus::Running: return "Running";
                        case ngc::SimulationStatus::Paused: return "Paused";
                        case ngc::SimulationStatus::Completed: return "Completed";
                        case ngc::SimulationStatus::Error: return "Error";
                    }
                    return "Unknown";
                }();
                ImGui::Text("Simulation: %s  XYZ: %.4f, %.4f, %.4f",
                            statusText, simulation.toolPosition.x, simulation.toolPosition.y, simulation.toolPosition.z);
                if(!simulation.error.empty()) {
                    ImGui::TextColored(ImVec4_Redish, "%s", simulation.error.c_str());
                }

                if(ImGui::BeginTabBar("programs")) {
                    for(auto &[source, name] : m_programSource) {
                        if(ImGui::BeginTabItem(name.c_str())) {
                            ImGui::InputTextMultiline("##source", &source, { -1, -1 });
                            ImGui::EndTabItem();
                        }
                    }
                }

                ImGui::EndTabBar();
            }

            ImGui::EndChild();

            if(ImGui::BeginChild("middle", {0, 0 }, ImGuiChildFlags_Border | ImGuiChildFlags_ResizeY)) {
                for(const auto &error : m_worker.parserErrors()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
                    ImGui::Selectable(error.text().c_str());
                    ImGui::PopStyleColor();
                }
            }

            ImGui::EndChild();

            if(ImGui::BeginChild("bottom", {0, 0 }, ImGuiChildFlags_Border)) {
                for(const auto &block : m_worker.blockMessages()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Greenish);
                    ImGui::Selectable(block.c_str());
                    ImGui::PopStyleColor();
                }
            }

            ImGui::EndChild();
        }

        ImGui::End();
    }

    void renderOpenDialog() {
        ImGui::OpenPopup("Open File");

        if(ImGui::BeginPopupModal("Open File", &m_enableOpenDialog)) {
            ImGui::Text("path: %s", m_path.string().c_str());
            ImGui::Separator();

            if(ImGui::Selectable("..")) {
                m_path = (m_path / "..").lexically_normal();
            }

            std::error_code directoryError;
            std::filesystem::directory_iterator iterator(m_path, directoryError);
            for(; !directoryError && iterator != std::filesystem::directory_iterator(); iterator.increment(directoryError)) {
                const auto &entry = *iterator;
                if(ImGui::Selectable(std::format("{}", entry.path().filename().string()).c_str())) {
                    std::error_code typeError;
                    if(entry.is_directory(typeError)) {
                        m_path = entry.path().lexically_normal();
                        break;
                    }

                    if(!typeError && entry.is_regular_file(typeError)) {
                        auto result = ngc::readFile(entry.path().string());

                        if(!result) {
                            m_errorMessage = result.error().what();
                            m_enableErrorWindow = true;
                            break;
                        }

                        auto autoload = readAutoloadSources();
                        if(!autoload) {
                            m_errorMessage = autoload.error();
                            m_enableErrorWindow = true;
                            break;
                        }

                        m_autoloadSource = std::move(*autoload);
                        auto programs = m_autoloadSource;
                        programs.emplace_back(*result, entry.path().string());
                        m_programSource = std::move(programs);
                        rebuildMainProgramLines();
                        m_enableOpenDialog = false;
                        m_programPaneMode = ProgramPaneMode::Edit;
                        m_programDirty = false;
                        m_errorMessage.clear();
                    }

                    if(typeError) {
                        m_errorMessage = std::format("failed to inspect '{}': {}", entry.path().string(), typeError.message());
                        m_enableErrorWindow = true;
                        break;
                    }
                }
            }

            if(directoryError) {
                m_errorMessage = std::format("failed to read '{}': {}", m_path.string(), directoryError.message());
                m_enableErrorWindow = true;
            }

            ImGui::EndPopup();
        }
    }
};

Application::Application(GLFWwindow *window) : m_impl(std::make_unique<ApplicationImpl>(window)) { }
Application::~Application() = default;

void Application::init() { m_impl->init(); }
void Application::preRender() { m_impl->preRender(); }
void Application::addScroll(const double delta) { m_impl->addScroll(delta); }
void Application::terminate() { m_impl->terminate(); }
void Application::render() { m_impl->render(); }
void Application::render3D() { m_impl->render3D(); }
