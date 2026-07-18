#include "Application.h"

#include <filesystem>
#include <fstream>
#include <format>
#include <algorithm>
#include <array>
#include <atomic>
#include <print>
#include <string>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <ranges>
#include <numbers>
#include <numeric>
#include <limits>
#include <optional>
#include <utility>
#include <iterator>
#include <sstream>
#include <unordered_set>
#include <thread>

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
    using GlGenBuffersProc = void (APIENTRY *)(GLsizei, GLuint *);
    using GlDeleteBuffersProc = void (APIENTRY *)(GLsizei, const GLuint *);
    using GlBindBufferProc = void (APIENTRY *)(GLenum, GLuint);
    using GlBufferDataProc = void (APIENTRY *)(GLenum, std::ptrdiff_t, const void *, GLenum);
    static constexpr GLenum GL_ARRAY_BUFFER_VALUE = 0x8892;
    static constexpr GLenum GL_STATIC_DRAW_VALUE = 0x88E4;

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
    float m_jogPaneWidth = 270.0f;
    ImVec4 m_viewportRect { 0, 0, 0, 0 };
    double m_lastFrameRenderSeconds = 0.0;
    bool m_showJogPane = false;
    int m_jogTargetMode = 0;
    bool m_continuousJog = true;
    bool m_individualJogEnabled = false;
    double m_jogSpeedPercent = 25.0;
    int m_jogStepIndex = 2;
    ngc::RequestId m_nextJogRequest = 1000;
    ngc::JogId m_nextJogId = 1;
    std::optional<ngc::JogId> m_uiContinuousJog;
    double m_lastJogRenewal = 0.0;

    // windows
    bool m_enableOpenDialog = false;
    bool m_enableProgramWindow = false; // Legacy window is no longer opened; retained until its renderer is removed.
    bool m_enableMemoryWindow = false;
    bool m_enableToolWindow = false;
    bool m_enableErrorWindow = false;
    bool m_showClusterGeometricJerkComb = false;
    bool m_showExecutedJerkComb = true;

    std::string m_errorMessage;

    ngc::SimulationTiming m_simulationTiming;
    ngc::JoggingConfiguration m_joggingConfiguration;
    ngc::Machine::Unit m_machineUnit;
    std::vector<ngc::AxisConfiguration> m_axes;
    std::vector<ngc::JointConfiguration> m_joints;
    Worker m_worker;
    SimulationWorker m_simulation;
    int m_simulationTickMultiplier = 1;
    double m_simulatedRapidSpeed;
    double m_pathJerk;

    double m_time = 0.0;
    double m_dt = 0.0;
    double m_mouseX = 0.0;
    double m_mouseY = 0.0;
    double m_mouseDX = 0.0;
    double m_mouseDY = 0.0;
    double m_scrollDelta = 0.0;
    bool m_rotationDragHasPivot = false;

    glm::dquat m_modelOrientation { 1.0, 0.0, 0.0, 0.0 };
    glm::dvec2 m_viewPan { 0.0, 0.0 };
    double m_viewHalfHeight = 6.0;

    enum class PreviewBatch : std::size_t {
        Feed, Rapid, G53Feed, G53Rapid, Arc, Probe, G64Spline, G64Cluster,
        G64ControlPolygon, G64ControlPoints, DarkPoints, LightPoints, Count,
    };

    struct BufferedRange {
        std::size_t first = 0;
        std::size_t count = 0;
    };

    struct GeometricJerkCombSample {
        glm::dvec3 position{};
        glm::dvec3 normalDirection{};
        double magnitude = 0.0;
        double normalMagnitude = 0.0;
        double tangentialMagnitude = 0.0;
        double geometricSpeedLimit = std::numeric_limits<double>::infinity();
        double programmedSpeed = 0.0;
    };

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
        std::vector<GeometricJerkCombSample> clusterGeometricJerkComb;
        double maximumClusterGeometricJerk = 0.0;
        std::vector<glm::dvec3> darkPoints;
        std::vector<glm::dvec3> lightPoints;
        std::optional<glm::dvec3> visibleCentroid;
        glm::dvec3 sceneMinimum{};
        glm::dvec3 sceneMaximum{};
        bool hasSceneGeometry = false;
        std::array<BufferedRange, static_cast<std::size_t>(PreviewBatch::Count)> bufferedRanges{};
    } m_previewRenderCache;
    struct PreviewViewSnapshot {
        glm::dquat orientation { 1.0, 0.0, 0.0, 0.0 };
        glm::dvec3 pivot{};
        glm::dvec2 pan{};
        double halfHeight = 1.0;
        double aspect = 1.0;
        int viewportWidth = 1;
        int viewportHeight = 1;
        std::uint64_t previewRevision = 0;
        std::uint64_t generation = 0;
    };
    std::mutex m_previewVisibilityMutex;
    std::condition_variable m_previewVisibilityCv;
    std::thread m_previewVisibilityThread;
    std::atomic<std::uint64_t> m_latestPreviewGeneration{0};
    std::atomic_bool m_stopPreviewVisibilityRequested{false};
    PreviewViewSnapshot m_requestedPreviewView;
    bool m_hasRequestedPreviewView = false;
    bool m_stopPreviewVisibility = false;
    std::optional<PreviewRenderCache> m_readyPreviewRenderCache;
    std::uint64_t m_readyPreviewGeneration = 0;
    struct ExecutedJerkTooth {
        glm::dvec3 position{};
        glm::dvec3 normal{};
        double utilization = 0.0;
    };
    struct PendingJerkObservation {
        glm::dvec3 position{};
        double utilization = 0.0;
    };
    std::vector<ExecutedJerkTooth> m_executedJerkComb;
    std::optional<PendingJerkObservation> m_pendingJerkObservation;
    std::uint32_t m_executedJerkServoPeriodsSinceTooth = 0;
    ngc::SimulationStatus m_lastJerkCombStatus = ngc::SimulationStatus::Stopped;
    std::uint64_t m_lastJerkCombServoTicks = 0;
    GlGenBuffersProc m_glGenBuffers = nullptr;
    GlDeleteBuffersProc m_glDeleteBuffers = nullptr;
    GlBindBufferProc m_glBindBuffer = nullptr;
    GlBufferDataProc m_glBufferData = nullptr;
    GLuint m_previewGeometryBuffer = 0;
    double m_sceneScale = 1.0;
    glm::dvec3 m_modelPivot = { 0.0, 0.0, 0.0 };

    static constexpr double MACHINE_TRIAD_LENGTH = 2.0;
    static constexpr double WORK_TRIAD_LENGTH = 1.0;
    static constexpr auto PREVIEW_VISIBILITY_PERIOD = std::chrono::milliseconds(100);
    static constexpr double PREVIEW_VISIBILITY_MARGIN = 0.12;
    static constexpr double PREVIEW_CURVE_ERROR_PIXELS = 0.5;
    static constexpr std::size_t PREVIEW_CURVE_MAXIMUM_SUBDIVISION_NODES = 1U << 20;

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
        glColor3d(1.0, 0.85, 0.05);
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

    void drawNoToolCrosshair(const ngc::position_t &position) const {
        const glm::dvec3 center { position.x, position.y, position.z };
        // World-space size is intentional: the marker grows when the user zooms in.
        const auto halfSize = m_machineUnit == ngc::Machine::Unit::Inch ? 0.5 : 12.5;
        const glm::dvec3 x { halfSize, 0.0, 0.0 };
        const glm::dvec3 y { 0.0, halfSize, 0.0 };
        const glm::dvec3 z { 0.0, 0.0, halfSize };

        glDisable(GL_DEPTH_TEST);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        glColor3d(1.0, 0.25, 0.2);
        glVertex3dv(glm::value_ptr(center - x));
        glVertex3dv(glm::value_ptr(center + x));
        glColor3d(0.25, 1.0, 0.3);
        glVertex3dv(glm::value_ptr(center - y));
        glVertex3dv(glm::value_ptr(center + y));
        glColor3d(0.25, 0.55, 1.0);
        glVertex3dv(glm::value_ptr(center - z));
        glVertex3dv(glm::value_ptr(center + z));
        glEnd();
        glLineWidth(1.0f);
    }

public:
    ApplicationImpl() = delete;
    ApplicationImpl(GLFWwindow *window, const ngc::MachineConfiguration &configuration)
        : m_window(window), m_simulationTiming(configuration.simulation),
          m_joggingConfiguration(configuration.jogging), m_machineUnit(configuration.unit),
          m_axes(configuration.axes), m_joints(configuration.joints),
          m_worker(configuration.unit),
          m_simulation(configuration),
          m_simulatedRapidSpeed(configuration.trajectory.rapidSpeed),
          m_pathJerk(configuration.trajectory.pathJerk) { }

    void init() {
        m_glGenBuffers = reinterpret_cast<GlGenBuffersProc>(glfwGetProcAddress("glGenBuffers"));
        m_glDeleteBuffers = reinterpret_cast<GlDeleteBuffersProc>(glfwGetProcAddress("glDeleteBuffers"));
        m_glBindBuffer = reinterpret_cast<GlBindBufferProc>(glfwGetProcAddress("glBindBuffer"));
        m_glBufferData = reinterpret_cast<GlBufferDataProc>(glfwGetProcAddress("glBufferData"));
        if(m_glGenBuffers && m_glDeleteBuffers && m_glBindBuffer && m_glBufferData) {
            m_glGenBuffers(1, &m_previewGeometryBuffer);
        }
        m_previewVisibilityThread = std::thread(&ApplicationImpl::previewVisibilityWork, this);
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

    void setLastFrameRenderSeconds(const double seconds) {
        m_lastFrameRenderSeconds=std::max(seconds,0.0);
    }

    void addScroll(const double delta) {
        m_scrollDelta += delta;
    }

    void terminate() {
        m_stopPreviewVisibilityRequested.store(true, std::memory_order_release);
        {
            std::scoped_lock lock(m_previewVisibilityMutex);
            m_stopPreviewVisibility = true;
        }
        m_previewVisibilityCv.notify_one();
        if(m_previewVisibilityThread.joinable()) m_previewVisibilityThread.join();
        m_simulation.join();
        m_worker.join();
        if(m_glDeleteBuffers) {
            m_glDeleteBuffers(1, &m_previewGeometryBuffer);
            m_previewGeometryBuffer = 0;
        }
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
        auto minimum = m_previewRenderCache.hasSceneGeometry
            ? m_previewRenderCache.sceneMinimum : glm::dvec3(-1.0);
        auto maximum = m_previewRenderCache.hasSceneGeometry
            ? m_previewRenderCache.sceneMaximum : glm::dvec3(1.0);

        m_modelPivot = (minimum + maximum) * 0.5;
        m_viewPan = { 0.0, 0.0 };
        const auto radius = std::max(glm::length(maximum - minimum) * 0.5, 0.1);
        m_sceneScale = radius;
        const auto aspect = static_cast<double>(std::max(width, 1)) / std::max(height, 1);
        m_viewHalfHeight = radius * 1.1 / std::min(aspect, 1.0);
    }

    bool pickToolpathRotationPivot(const int width, const int height,
                                   const int viewportLeft, const int viewportTop) {
        constexpr double PICK_RADIUS_PIXELS=30.0;
        const auto aspect=static_cast<double>(width)/height;
        const glm::dvec2 mouse{m_mouseX-viewportLeft,m_mouseY-viewportTop};
        auto bestDistanceSquared=PICK_RADIUS_PIXELS*PICK_RADIUS_PIXELS;
        std::optional<glm::dvec3> bestPoint;
        const auto project=[&](const glm::dvec3 &point) {
            const auto transformed=m_modelOrientation*displayOffset(point-m_modelPivot);
            const auto ndcX=(transformed.x+m_viewPan.x)/(m_viewHalfHeight*aspect);
            const auto ndcY=(transformed.z+m_viewPan.y)/m_viewHalfHeight;
            return glm::dvec2((ndcX+1.0)*width*0.5,(1.0-ndcY)*height*0.5);
        };
        const auto consider=[&](const glm::dvec3 &from,const glm::dvec3 &to) {
            const auto screenFrom=project(from);
            const auto screenTo=project(to);
            const auto screenSegment=screenTo-screenFrom;
            const auto lengthSquared=glm::dot(screenSegment,screenSegment);
            const auto parameter=lengthSquared>0.0
                ?std::clamp(glm::dot(mouse-screenFrom,screenSegment)/lengthSquared,0.0,1.0)
                :0.0;
            const auto difference=mouse-glm::mix(screenFrom,screenTo,parameter);
            const auto distanceSquared=glm::dot(difference,difference);
            if(distanceSquared>=bestDistanceSquared) return;
            bestDistanceSquared=distanceSquared;
            bestPoint=glm::mix(from,to,parameter);
        };
        const std::array toolpathLines{
            &m_previewRenderCache.feedLines,&m_previewRenderCache.rapidLines,
            &m_previewRenderCache.g53FeedLines,&m_previewRenderCache.g53RapidLines,
            &m_previewRenderCache.arcLines,&m_previewRenderCache.probeLines,
            &m_previewRenderCache.g64SplineLines,
            &m_previewRenderCache.g64ClusterSplineLines,
        };
        for(const auto *lines:toolpathLines)
            for(std::size_t index=1;index<lines->size();index+=2)
                consider((*lines)[index-1],(*lines)[index]);
        if(!bestPoint) return false;

        // Preserve the current screen transform while making the selected
        // toolpath point the sole pivot for this rotation drag.
        const auto compensation=m_modelOrientation*displayOffset(*bestPoint-m_modelPivot);
        m_viewPan.x+=compensation.x;
        m_viewPan.y+=compensation.z;
        m_modelPivot=*bestPoint;
        return true;
    }

    std::expected<PreviewRenderCache, std::string> buildPreviewGeometry(
            const ngc::PreparedPreviewScene &preparedScene,
            const PreviewViewSnapshot &view,
            const bool rejectOutsideView) const {
            PreviewRenderCache cache;
            std::size_t subdivisionNodes = 0;

            const auto cancelled = [&] {
                return m_stopPreviewVisibilityRequested.load(std::memory_order_acquire)
                    || m_latestPreviewGeneration.load(std::memory_order_acquire)
                        != view.generation;
            };

            const auto segment = [](auto &vertices, const glm::dvec3 &from, const glm::dvec3 &to) {
                vertices.push_back(from);
                vertices.push_back(to);
            };

            const auto offsetPoint = [](const ngc::position_t &value,
                                        const ngc::position_t &offset) {
                return glm::dvec3(value.x - offset.x, value.y - offset.y,
                                  value.z - offset.z);
            };
            const auto commandOffset = [](const ngc::PreparedGeometrySlice &slice,
                                          const ngc::PreparedCommandId id) {
                const auto found = std::ranges::find_if(slice.commands,
                    [id](const auto &command) { return command.id == id; });
                return found == slice.commands.end()
                    ? ngc::position_t{} : found->presentation.activeToolOffset;
            };
            ngc::CurveEvaluationWorkspace curveWorkspace;
            const auto projectToPixels = [&](const glm::dvec3 &point) {
                const auto transformed = view.orientation * displayOffset(point - view.pivot);
                const auto ndcX = (transformed.x + view.pan.x)
                    / (view.halfHeight * view.aspect);
                const auto ndcY = (transformed.z + view.pan.y) / view.halfHeight;
                return glm::dvec2(ndcX * view.viewportWidth * 0.5,
                                  ndcY * view.viewportHeight * 0.5);
            };
            const auto appendCurve = [&](const ngc::PreparedCurve &curve,
                                         const double fromDistance,
                                         const double toDistance,
                                         const ngc::position_t &offset,
                                         std::vector<glm::dvec3> &vertices,
                                         const bool adaptive)
                    -> std::expected<std::pair<glm::dvec3, glm::dvec3>, std::string> {
                const auto position = [&](const double distance) {
                    return offsetPoint(ngc::positionAtDistance(
                        curve, distance, curveWorkspace), offset);
                };
                if(cancelled()) return std::unexpected("cancelled");
                const auto beginning = position(fromDistance);
                const auto ending = position(toDistance);
                if(!adaptive) {
                    segment(vertices, beginning, ending);
                    return std::pair { beginning, ending };
                }
                const auto pixelsPerWorldUnit = std::max(
                    view.viewportWidth / (2.0 * view.halfHeight * view.aspect),
                    view.viewportHeight / (2.0 * view.halfHeight));
                const auto regionOutsideView = [&](const glm::dvec3 &fromPoint,
                                                   const glm::dvec3 &toPoint,
                                                   const double screenError) {
                    if(!rejectOutsideView) return false;
                    const auto screenFrom = projectToPixels(fromPoint);
                    const auto screenTo = projectToPixels(toPoint);
                    const auto horizontalLimit = view.viewportWidth * 0.5
                        * (1.0 + PREVIEW_VISIBILITY_MARGIN);
                    const auto verticalLimit = view.viewportHeight * 0.5
                        * (1.0 + PREVIEW_VISIBILITY_MARGIN);
                    return std::max(screenFrom.x, screenTo.x) + screenError < -horizontalLimit
                        || std::min(screenFrom.x, screenTo.x) - screenError > horizontalLimit
                        || std::max(screenFrom.y, screenTo.y) + screenError < -verticalLimit
                        || std::min(screenFrom.y, screenTo.y) - screenError > verticalLimit;
                };
                const auto emit = [&](auto &&self, const double from,
                                      const glm::dvec3 &fromPoint, const double to,
                                      const glm::dvec3 &toPoint)
                        -> std::expected<void, std::string> {
                    if(cancelled()) return std::unexpected("cancelled");
                    if(++subdivisionNodes > PREVIEW_CURVE_MAXIMUM_SUBDIVISION_NODES)
                        return std::unexpected(std::format(
                            "preview curve tessellation exceeded {} subdivision nodes",
                            PREVIEW_CURVE_MAXIMUM_SUBDIVISION_NODES));
                    const auto worldError = ngc::chordErrorBound(
                        curve, from, to, curveWorkspace);
                    // Orthographic projection cannot amplify a spatial error by more than
                    // the largest pixels-per-world-unit scale. This makes the prepared
                    // curve's conservative chord bound the screen-space acceptance proof.
                    const auto screenError = worldError * pixelsPerWorldUnit;
                    if(!std::isfinite(screenError) || screenError < 0.0)
                        return std::unexpected(
                            "preview curve tessellation produced a non-finite error bound");
                    if(regionOutsideView(fromPoint, toPoint, screenError)) return {};
                    if(screenError <= PREVIEW_CURVE_ERROR_PIXELS) {
                        segment(vertices, fromPoint, toPoint);
                        return {};
                    }
                    const auto middleDistance = std::midpoint(from, to);
                    if(!(middleDistance > from && middleDistance < to))
                        return std::unexpected(
                            "preview curve tessellation exhausted floating-point distance resolution");
                    const auto middlePoint = position(middleDistance);
                    if(auto left = self(self, from, fromPoint, middleDistance, middlePoint); !left)
                        return left;
                    return self(self, middleDistance, middlePoint, to, toPoint);
                };
                if(auto emitted = emit(emit, fromDistance, beginning, toDistance, ending); !emitted)
                    return std::unexpected(emitted.error());
                return std::pair { beginning, ending };
            };
            for(const auto &slice : preparedScene.continuousSlices) {
                for(const auto &piece : slice.pieces) {
                    if(cancelled()) return std::unexpected("cancelled");
                    if(!piece.curve) continue;
                    const auto offset = commandOffset(slice, piece.primaryCommand);
                    auto *vertices = piece.kind == ngc::PreparedPieceKind::ClusterSpline
                        ? &cache.g64ClusterSplineLines
                        : piece.kind == ngc::PreparedPieceKind::JunctionBlend
                            ? &cache.g64SplineLines
                            : piece.kind == ngc::PreparedPieceKind::RetainedArcSection
                                ? &cache.arcLines : &cache.feedLines;
                    const auto appended = appendCurve(
                        *piece.curve, piece.curveFrom, piece.curveTo, offset, *vertices,
                        piece.kind != ngc::PreparedPieceKind::RetainedLineSection);
                    if(!appended) return std::unexpected(appended.error());
                    const auto &[beginning, ending] = *appended;
                    cache.darkPoints.push_back(beginning);
                    cache.lightPoints.push_back(ending);

                    if(const auto *spline = std::get_if<ngc::PreparedSplineCurve>(
                            &piece.curve->value)) {
                        for(const auto &control : spline->controls)
                            cache.g64ControlPoints.push_back(offsetPoint(control, offset));
                        for(std::size_t control = 1; control < spline->controls.size(); ++control)
                            segment(cache.g64ControlPolygon,
                                offsetPoint(spline->controls[control - 1], offset),
                                offsetPoint(spline->controls[control], offset));
                    }
                    if(piece.kind != ngc::PreparedPieceKind::ClusterSpline) continue;
                    for(const auto &sample : piece.geometricSamples) {
                        const auto curvature = glm::dvec3(sample.curvature.x,
                            sample.curvature.y, sample.curvature.z);
                        const auto curvatureLength = glm::length(curvature);
                        GeometricJerkCombSample displaySample;
                        displaySample.position = offsetPoint(sample.position, offset);
                        if(curvatureLength > 1e-15)
                            displaySample.normalDirection = curvature / curvatureLength;
                        displaySample.magnitude = sample.fullGeometricJerkCoefficient;
                        displaySample.normalMagnitude = sample.normalSharpness;
                        displaySample.tangentialMagnitude = curvatureLength * curvatureLength;
                        displaySample.programmedSpeed = piece.programmedFeed;
                        if(m_pathJerk > 0.0 && displaySample.magnitude > 1e-15)
                            displaySample.geometricSpeedLimit = std::cbrt(
                                m_pathJerk / displaySample.magnitude);
                        cache.maximumClusterGeometricJerk = std::max(
                            cache.maximumClusterGeometricJerk, displaySample.magnitude);
                        cache.clusterGeometricJerkComb.push_back(displaySample);
                    }
                }
            }

            for(const auto &standalone : preparedScene.standaloneCommands) {
                if(cancelled()) return std::unexpected("cancelled");
                if(!standalone.displayGeometry) continue;
                const auto &command = standalone.command.command;
                auto *vertices = &cache.feedLines;
                auto showEndpoints = true;
                if(const auto *line = std::get_if<ngc::MoveLine>(&command)) {
                    const auto rapid = line->speed() == -1.0;
                    if(line->machineCoordinates())
                        vertices = rapid ? &cache.g53RapidLines : &cache.g53FeedLines;
                    else if(rapid) vertices = &cache.rapidLines;
                } else if(std::holds_alternative<ngc::MoveArc>(command)) {
                    vertices = &cache.arcLines;
                } else if(std::holds_alternative<ngc::ProbeMove>(command)) {
                    vertices = &cache.probeLines;
                    showEndpoints = false;
                } else continue;

                const auto &curve = *standalone.displayGeometry;
                const auto offset = standalone.command.presentation.activeToolOffset;
                const auto appended = appendCurve(
                    curve, 0.0, curve.length, offset, *vertices,
                    std::holds_alternative<ngc::MoveArc>(command));
                if(!appended) return std::unexpected(appended.error());
                const auto &[beginning, ending] = *appended;
                if(showEndpoints) {
                    cache.darkPoints.push_back(beginning);
                    cache.lightPoints.push_back(ending);
                }
            }

            cache.revision = preparedScene.revision;
            const std::array geometry {
                &cache.feedLines, &cache.rapidLines, &cache.g53FeedLines,
                &cache.g53RapidLines, &cache.arcLines, &cache.probeLines,
                &cache.g64SplineLines, &cache.g64ClusterSplineLines,
            };
            for(const auto *vertices : geometry) {
                for(const auto &vertex : *vertices) {
                    if(!cache.hasSceneGeometry) {
                        cache.sceneMinimum = vertex;
                        cache.sceneMaximum = vertex;
                        cache.hasSceneGeometry = true;
                    } else {
                        cache.sceneMinimum = glm::min(cache.sceneMinimum, vertex);
                        cache.sceneMaximum = glm::max(cache.sceneMaximum, vertex);
                    }
                }
            }
            if(cancelled()) return std::unexpected("cancelled");
            return cache;
    }

    static std::optional<std::pair<double, double>> clipPreviewSegment(
        const glm::dvec2 &from, const glm::dvec2 &to, const double limit) {
        auto first = 0.0;
        auto last = 1.0;
        const auto delta = to - from;
        const auto clip = [&](const double direction, const double distance) {
            if(std::abs(direction) < 1e-15) return distance >= 0.0;
            const auto parameter = distance / direction;
            if(direction < 0.0) {
                if(parameter > last) return false;
                first = std::max(first, parameter);
            } else {
                if(parameter < first) return false;
                last = std::min(last, parameter);
            }
            return true;
        };
        if(!clip(-delta.x, from.x + limit)
           || !clip(delta.x, limit - from.x)
           || !clip(-delta.y, from.y + limit)
           || !clip(delta.y, limit - from.y)) return std::nullopt;
        return std::pair { first, last };
    }

    std::optional<PreviewRenderCache> cullPreviewGeometry(
            const PreviewRenderCache &source,
            const PreviewViewSnapshot &view) const {
        PreviewRenderCache visible;
        const auto cancelled = [&] {
            return m_stopPreviewVisibilityRequested.load(std::memory_order_acquire)
                || m_latestPreviewGeneration.load(std::memory_order_acquire)
                    != view.generation;
        };
        visible.revision = source.revision;
        visible.sceneMinimum = source.sceneMinimum;
        visible.sceneMaximum = source.sceneMaximum;
        visible.hasSceneGeometry = source.hasSceneGeometry;
        const auto project = [&](const glm::dvec3 &point) {
            const auto transformed = view.orientation * displayOffset(point - view.pivot);
            return glm::dvec2(
                (transformed.x + view.pan.x) / (view.halfHeight * view.aspect),
                (transformed.z + view.pan.y) / view.halfHeight);
        };

        glm::dvec3 weightedCentroid{};
        auto centroidWeight = 0.0;
        const auto cullLines = [&](const std::vector<glm::dvec3> &input,
                                   std::vector<glm::dvec3> &output,
                                   const bool includeInCentroid) {
            output.reserve(input.size());
            for(std::size_t index = 1; index < input.size(); index += 2) {
                if(cancelled()) return false;
                const auto &from = input[index - 1];
                const auto &to = input[index];
                const auto projectedFrom = project(from);
                const auto projectedTo = project(to);
                if(!clipPreviewSegment(projectedFrom, projectedTo,
                        1.0 + PREVIEW_VISIBILITY_MARGIN)) continue;
                output.push_back(from);
                output.push_back(to);
                if(!includeInCentroid) continue;
                const auto clipped = clipPreviewSegment(projectedFrom, projectedTo, 1.0);
                if(!clipped) continue;
                const auto [first, last] = *clipped;
                const auto clippedFrom = glm::mix(projectedFrom, projectedTo, first);
                const auto clippedTo = glm::mix(projectedFrom, projectedTo, last);
                const auto weight = glm::length(clippedTo - clippedFrom);
                if(weight <= 1e-15) continue;
                weightedCentroid += glm::mix(from, to, (first + last) * 0.5) * weight;
                centroidWeight += weight;
            }
            return true;
        };
        const auto cullPoints = [&](const std::vector<glm::dvec3> &input,
                                    std::vector<glm::dvec3> &output) {
            output.reserve(input.size());
            for(const auto &point : input) {
                if(cancelled()) return false;
                const auto projected = project(point);
                if(std::abs(projected.x) <= 1.0 + PREVIEW_VISIBILITY_MARGIN
                   && std::abs(projected.y) <= 1.0 + PREVIEW_VISIBILITY_MARGIN)
                    output.push_back(point);
            }
            return true;
        };

        if(!cullLines(source.feedLines, visible.feedLines, true)
           ||!cullLines(source.rapidLines, visible.rapidLines, true)
           ||!cullLines(source.g53FeedLines, visible.g53FeedLines, true)
           ||!cullLines(source.g53RapidLines, visible.g53RapidLines, true)
           ||!cullLines(source.arcLines, visible.arcLines, true)
           ||!cullLines(source.probeLines, visible.probeLines, true)
           ||!cullLines(source.g64SplineLines, visible.g64SplineLines, true)
           ||!cullLines(source.g64ClusterSplineLines, visible.g64ClusterSplineLines, true)
           ||!cullLines(source.g64ControlPolygon, visible.g64ControlPolygon, false)
           ||!cullPoints(source.g64ControlPoints, visible.g64ControlPoints)
           ||!cullPoints(source.darkPoints, visible.darkPoints)
           ||!cullPoints(source.lightPoints, visible.lightPoints)) return std::nullopt;
        if(centroidWeight > 0.0) visible.visibleCentroid = weightedCentroid / centroidWeight;

        visible.clusterGeometricJerkComb.reserve(source.clusterGeometricJerkComb.size());
        for(const auto &sample : source.clusterGeometricJerkComb) {
            if(cancelled()) return std::nullopt;
            const auto projected = project(sample.position);
            if(std::abs(projected.x) > 1.0 + PREVIEW_VISIBILITY_MARGIN
               || std::abs(projected.y) > 1.0 + PREVIEW_VISIBILITY_MARGIN) continue;
            visible.maximumClusterGeometricJerk = std::max(
                visible.maximumClusterGeometricJerk, sample.magnitude);
            visible.clusterGeometricJerkComb.push_back(sample);
        }
        return visible;
    }

    void previewVisibilityWork() {
        ngc::PreparedPreviewScene scene;
        bool hasScene = false;
        bool hasCompleteSceneBounds = false;
        glm::dvec3 completeSceneMinimum{};
        glm::dvec3 completeSceneMaximum{};
        std::uint64_t processedGeneration = 0;
        auto nextBuild = std::chrono::steady_clock::now();
        for(;;) {
            PreviewViewSnapshot view;
            {
                std::unique_lock lock(m_previewVisibilityMutex);
                m_previewVisibilityCv.wait(lock, [&] {
                    return m_stopPreviewVisibility
                        || (m_hasRequestedPreviewView
                            && m_requestedPreviewView.generation != processedGeneration);
                });
                if(m_stopPreviewVisibility) return;
                const auto now = std::chrono::steady_clock::now();
                if(now < nextBuild) {
                    m_previewVisibilityCv.wait_until(lock, nextBuild,
                        [&] { return m_stopPreviewVisibility; });
                    if(m_stopPreviewVisibility) return;
                }
                view = m_requestedPreviewView;
            }

            if(!hasScene || scene.revision != view.previewRevision) {
                scene = m_worker.lock([&] { return m_worker.preparedPreview(); });
                hasScene = true;
                hasCompleteSceneBounds = false;
                view.previewRevision = scene.revision;
            }
            auto source = buildPreviewGeometry(
                scene, view, hasCompleteSceneBounds);
            if(!source) {
                if(source.error() != "cancelled")
                    std::println(stderr, "Preview geometry rebuild failed: {}", source.error());
                std::scoped_lock lock(m_previewVisibilityMutex);
                processedGeneration = view.generation;
                continue;
            }
            if(!hasCompleteSceneBounds && source->hasSceneGeometry) {
                completeSceneMinimum = source->sceneMinimum;
                completeSceneMaximum = source->sceneMaximum;
                hasCompleteSceneBounds = true;
            } else if(hasCompleteSceneBounds) {
                source->sceneMinimum = completeSceneMinimum;
                source->sceneMaximum = completeSceneMaximum;
                source->hasSceneGeometry = true;
            }
            auto visible = cullPreviewGeometry(*source, view);
            if(!visible) {
                std::scoped_lock lock(m_previewVisibilityMutex);
                processedGeneration = view.generation;
                continue;
            }
            {
                std::scoped_lock lock(m_previewVisibilityMutex);
                if(m_stopPreviewVisibility) return;
                if(view.generation >= m_readyPreviewGeneration) {
                    m_readyPreviewGeneration = view.generation;
                    m_readyPreviewRenderCache = std::move(*visible);
                }
                processedGeneration = view.generation;
            }
            nextBuild = std::chrono::steady_clock::now() + PREVIEW_VISIBILITY_PERIOD;
        }
    }

    void requestPreviewVisibility(const double aspect, const int width, const int height) {
        const auto revision = m_worker.lock([&] { return m_worker.preparedPreview().revision; });
        std::scoped_lock lock(m_previewVisibilityMutex);
        const auto changed = !m_hasRequestedPreviewView
            || m_requestedPreviewView.orientation != m_modelOrientation
            || m_requestedPreviewView.pivot != m_modelPivot
            || m_requestedPreviewView.pan != m_viewPan
            || m_requestedPreviewView.halfHeight != m_viewHalfHeight
            || m_requestedPreviewView.aspect != aspect
            || m_requestedPreviewView.viewportWidth != width
            || m_requestedPreviewView.viewportHeight != height
            || m_requestedPreviewView.previewRevision != revision;
        if(!changed) return;
        m_requestedPreviewView.orientation = m_modelOrientation;
        m_requestedPreviewView.pivot = m_modelPivot;
        m_requestedPreviewView.pan = m_viewPan;
        m_requestedPreviewView.halfHeight = m_viewHalfHeight;
        m_requestedPreviewView.aspect = aspect;
        m_requestedPreviewView.viewportWidth = width;
        m_requestedPreviewView.viewportHeight = height;
        m_requestedPreviewView.previewRevision = revision;
        ++m_requestedPreviewView.generation;
        m_latestPreviewGeneration.store(
            m_requestedPreviewView.generation, std::memory_order_release);
        m_hasRequestedPreviewView = true;
        m_previewVisibilityCv.notify_one();
    }

    void consumePreviewVisibilityResult() {
        std::optional<PreviewRenderCache> ready;
        {
            std::scoped_lock lock(m_previewVisibilityMutex);
            if(!m_readyPreviewRenderCache) return;
            ready = std::move(m_readyPreviewRenderCache);
            m_readyPreviewRenderCache.reset();
        }
        if(ready->visibleCentroid&&!m_rotationDragHasPivot) {
            const auto compensation = m_modelOrientation
                * displayOffset(*ready->visibleCentroid - m_modelPivot);
            m_viewPan.x += compensation.x;
            m_viewPan.y += compensation.z;
            m_modelPivot = *ready->visibleCentroid;
        }
        m_previewRenderCache = std::move(*ready);
        if(!m_glBindBuffer || !m_glBufferData || m_previewGeometryBuffer == 0) return;

        std::vector<glm::dvec3> previewVertices;
        const std::array previewBatches {
            std::pair {PreviewBatch::Feed, &m_previewRenderCache.feedLines},
            std::pair {PreviewBatch::Rapid, &m_previewRenderCache.rapidLines},
            std::pair {PreviewBatch::G53Feed, &m_previewRenderCache.g53FeedLines},
            std::pair {PreviewBatch::G53Rapid, &m_previewRenderCache.g53RapidLines},
            std::pair {PreviewBatch::Arc, &m_previewRenderCache.arcLines},
            std::pair {PreviewBatch::Probe, &m_previewRenderCache.probeLines},
            std::pair {PreviewBatch::G64Spline, &m_previewRenderCache.g64SplineLines},
            std::pair {PreviewBatch::G64Cluster, &m_previewRenderCache.g64ClusterSplineLines},
            std::pair {PreviewBatch::G64ControlPolygon, &m_previewRenderCache.g64ControlPolygon},
            std::pair {PreviewBatch::G64ControlPoints, &m_previewRenderCache.g64ControlPoints},
            std::pair {PreviewBatch::DarkPoints, &m_previewRenderCache.darkPoints},
            std::pair {PreviewBatch::LightPoints, &m_previewRenderCache.lightPoints},
        };
        const auto previewVertexCount = std::accumulate(
            previewBatches.begin(), previewBatches.end(), std::size_t{},
            [](const std::size_t count, const auto &batch) { return count + batch.second->size(); });
        previewVertices.reserve(previewVertexCount);
        for(const auto &[batch, vertices] : previewBatches) {
            auto &range = m_previewRenderCache.bufferedRanges[static_cast<std::size_t>(batch)];
            range = { previewVertices.size(), vertices->size() };
            previewVertices.insert(previewVertices.end(), vertices->begin(), vertices->end());
        }
        m_glBindBuffer(GL_ARRAY_BUFFER_VALUE, m_previewGeometryBuffer);
        m_glBufferData(GL_ARRAY_BUFFER_VALUE,
                       static_cast<std::ptrdiff_t>(previewVertices.size() * sizeof(glm::dvec3)),
                       previewVertices.empty() ? nullptr : previewVertices.data(),
                       GL_STATIC_DRAW_VALUE);
        m_glBindBuffer(GL_ARRAY_BUFFER_VALUE, 0);
    }

    void render3D() {
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

        if(ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) {
            m_rotationDragHasPivot=navigationActive&&!ctrl&&!shift
                &&pickToolpathRotationPivot(width,height,viewportLeft,viewportTop);
        }
        if(!middleDown) m_rotationDragHasPivot=false;

        if(navigationActive&&middleDown&&!ctrl&&!shift&&m_rotationDragHasPivot) {
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

        requestPreviewVisibility(aspect, width, height);
        consumePreviewVisibilityResult();

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
            CoordinateDisplay result {
                .active = {
                    .name = std::string(ngc::name(*machine.state().modeCoordSys)),
                    .offset = machine.workOffset(),
                },
                .used = {},
            };
            for(const auto &presentation : m_worker.preparedPreview().presentations) {
                if(!presentation.workCoordinateSystem) continue;
                const auto &candidate = *presentation.workCoordinateSystem;
                const auto found = std::ranges::find_if(result.used, [&](const auto &value) {
                    return value.name == candidate.name;
                });
                if(found == result.used.end()) result.used.push_back(candidate);
                else *found = candidate;
            }
            return result;
        });
        const auto simulationCoordinates = m_simulation.snapshot();
        const auto continuingStatus=m_lastJerkCombStatus==ngc::SimulationStatus::Running
            ||m_lastJerkCombStatus==ngc::SimulationStatus::Paused;
        const auto newSimulation=simulationCoordinates.status==ngc::SimulationStatus::Running
            &&(!continuingStatus
               ||simulationCoordinates.servoTicks<m_lastJerkCombServoTicks);
        if(newSimulation) {
            m_executedJerkComb.clear();
            m_pendingJerkObservation.reset();
            m_executedJerkServoPeriodsSinceTooth=0;
        }
        const auto executedJerkSamples=m_simulation.takeExecutedJerkSamples();
        if(m_showExecutedJerkComb&&m_pathJerk>0.0&&std::isfinite(m_pathJerk)) {
            for(const auto &sample:executedJerkSamples) {
                const glm::dvec3 position{
                    sample.position.x,sample.position.y,sample.position.z};
                const auto utilization=sample.magnitude/m_pathJerk;
                ++m_executedJerkServoPeriodsSinceTooth;
                if(!m_pendingJerkObservation) {
                    m_pendingJerkObservation=PendingJerkObservation{position,utilization};
                    continue;
                }
                const auto previous=*m_pendingJerkObservation;
                const auto delta=position-previous.position;
                const auto segmentLength=glm::length(delta);
                if(m_executedJerkServoPeriodsSinceTooth>=10) {
                    m_executedJerkServoPeriodsSinceTooth=0;
                    if(segmentLength<=1e-12) {
                        m_pendingJerkObservation=PendingJerkObservation{position,utilization};
                        continue;
                    }
                    const auto tangent=delta/segmentLength;
                    auto normal=glm::cross(glm::dvec3(0.0,0.0,1.0),tangent);
                    if(glm::length(normal)<=1e-12)
                        normal=glm::cross(glm::dvec3(0.0,1.0,0.0),tangent);
                    normal=glm::normalize(normal);
                    if(m_executedJerkComb.size()<100000)
                        m_executedJerkComb.push_back({position,normal,utilization});
                }
                m_pendingJerkObservation=PendingJerkObservation{position,utilization};
            }
        } else {
            m_pendingJerkObservation.reset();
            m_executedJerkServoPeriodsSinceTooth=0;
        }
        m_lastJerkCombStatus=simulationCoordinates.status;
        m_lastJerkCombServoTicks=simulationCoordinates.servoTicks;
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

        const auto drawTriad = [](const glm::dvec3 &origin, const double length, const bool dashed, const double intensity) {
            if(dashed) {
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(1, 0x3333);
            }

            glBegin(GL_LINES);
            glColor3d(intensity, 0.2 * intensity, 0.2 * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x + length, origin.y, origin.z);
            glColor3d(0.2 * intensity, intensity, 0.2 * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x, origin.y + length, origin.z);
            glColor3d(0.25 * intensity, 0.45 * intensity, intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glVertex3d(origin.x, origin.y, origin.z + length);
            glEnd();

            if(dashed) {
                glDisable(GL_LINE_STIPPLE);
            }

            glEnable(GL_POINT_SMOOTH);
            glPointSize(dashed ? 7.0f : 9.0f);
            glBegin(GL_POINTS);
            glColor3d(intensity, (dashed ? 0.75 : 0.95) * intensity,
                      (dashed ? 0.15 : 0.95) * intensity);
            glVertex3d(origin.x, origin.y, origin.z);
            glEnd();
        };

        glLineWidth(2.0f);
        drawTriad({ 0.0, 0.0, 0.0 }, MACHINE_TRIAD_LENGTH, false, 0.55);
        for(const auto &workCoordinateSystem : coordinates.used) {
            if(workCoordinateSystem.name == coordinates.active.name) continue;
            const auto &offset = workCoordinateSystem.offset;
            glLineWidth(2.0f);
            drawTriad({ offset.x, offset.y, offset.z }, WORK_TRIAD_LENGTH, true, 0.35);
        }
        glLineWidth(5.0f);
        drawTriad({ coordinates.active.offset.x, coordinates.active.offset.y, coordinates.active.offset.z },
                  WORK_TRIAD_LENGTH, true, 1.0);
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

        const auto renderTimeText=std::format(
            "Render: {:.2f} ms",m_lastFrameRenderSeconds*1000.0);
        const auto renderTimeSize=ImGui::CalcTextSize(renderTimeText.c_str());
        constexpr float RENDER_TIME_PADDING=8.0f;
        const ImVec2 renderTimePosition {
            m_viewportRect.z-renderTimeSize.x-RENDER_TIME_PADDING,
            m_viewportRect.w-renderTimeSize.y-RENDER_TIME_PADDING,
        };
        drawList->AddText({renderTimePosition.x+1.0f,renderTimePosition.y+1.0f},
                          IM_COL32(0,0,0,210),renderTimeText.c_str());
        drawList->AddText(renderTimePosition,IM_COL32(210,220,225,255),
                          renderTimeText.c_str());

        const auto drawVertices = [](const std::vector<glm::dvec3> &vertices, const GLenum primitive,
                                     const double red, const double green, const double blue,
                                     const int stippleFactor = 0, const GLushort stipplePattern = 0xFFFF) {
            if(vertices.empty()) return;
            if(stippleFactor > 0) {
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(stippleFactor, stipplePattern);
            }
            glColor3d(red, green, blue);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_DOUBLE, sizeof(glm::dvec3), &vertices.front().x);
            glDrawArrays(primitive, 0, static_cast<GLsizei>(vertices.size()));
            glDisableClientState(GL_VERTEX_ARRAY);
            if(stippleFactor > 0) glDisable(GL_LINE_STIPPLE);
        };
        const auto drawBufferedVertices = [&](const GLuint buffer, const std::size_t first,
                                              const std::size_t count,
                                              const GLenum primitive, const double red,
                                              const double green, const double blue,
                                              const int stippleFactor = 0,
                                              const GLushort stipplePattern = 0xFFFF) {
            if(buffer == 0 || count == 0 || !m_glBindBuffer) return false;
            if(stippleFactor > 0) {
                glEnable(GL_LINE_STIPPLE);
                glLineStipple(stippleFactor, stipplePattern);
            }
            glColor3d(red, green, blue);
            m_glBindBuffer(GL_ARRAY_BUFFER_VALUE, buffer);
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_DOUBLE, sizeof(glm::dvec3),
                            reinterpret_cast<const void *>(first * sizeof(glm::dvec3)));
            glDrawArrays(primitive, 0, static_cast<GLsizei>(count));
            glDisableClientState(GL_VERTEX_ARRAY);
            m_glBindBuffer(GL_ARRAY_BUFFER_VALUE, 0);
            if(stippleFactor > 0) glDisable(GL_LINE_STIPPLE);
            return true;
        };
        const auto drawPreview = [&](const PreviewBatch batch,
                                     const std::vector<glm::dvec3> &fallback,
                                     const GLenum primitive, const double red,
                                     const double green, const double blue,
                                     const int stippleFactor = 0,
                                     const GLushort stipplePattern = 0xFFFF) {
            const auto &range = m_previewRenderCache.bufferedRanges[static_cast<std::size_t>(batch)];
            if(!drawBufferedVertices(m_previewGeometryBuffer, range.first, range.count,
                                     primitive, red, green, blue, stippleFactor, stipplePattern))
                drawVertices(fallback, primitive, red, green, blue, stippleFactor, stipplePattern);
        };

        if(m_glBindBuffer) m_glBindBuffer(GL_ARRAY_BUFFER_VALUE, 0);
        glLineWidth(2.0f);
        drawPreview(PreviewBatch::Feed, m_previewRenderCache.feedLines, GL_LINES, 0.4f, 0.4f, 1.0f);
        drawPreview(PreviewBatch::G53Feed, m_previewRenderCache.g53FeedLines, GL_LINES, 1.0f, 1.0f, 0.4f);
        drawPreview(PreviewBatch::Arc, m_previewRenderCache.arcLines, GL_LINES, 0.4f, 1.0f, 0.4f);
        drawPreview(PreviewBatch::Probe, m_previewRenderCache.probeLines, GL_LINES, 1.0f, 0.4f, 0.2f);
        glLineWidth(1.0f);
        drawPreview(PreviewBatch::G64Spline, m_previewRenderCache.g64SplineLines, GL_LINES, 1.0f, 0.2f, 1.0f);
        drawPreview(PreviewBatch::G64Cluster, m_previewRenderCache.g64ClusterSplineLines, GL_LINES, 0.1f, 1.0f, 1.0f);
        if(m_showClusterGeometricJerkComb
           &&m_previewRenderCache.maximumClusterGeometricJerk>0.0) {
            const auto maximum=m_previewRenderCache.maximumClusterGeometricJerk;
            const auto logarithmicFloor=maximum*1e-6;
            const auto logarithmicMaximum=std::log1p(maximum/logarithmicFloor);
            const auto maximumToothLength=2.0*m_viewHalfHeight/height*55.0;
            glLineWidth(1.5f);
            glBegin(GL_LINES);
            for(const auto &sample:m_previewRenderCache.clusterGeometricJerkComb) {
                if(sample.magnitude<=0.0) continue;
                const auto normalized=std::log1p(sample.magnitude/logarithmicFloor)
                    /logarithmicMaximum;
                const auto toothLength=maximumToothLength*normalized;
                if(sample.geometricSpeedLimit+1e-12<sample.programmedSpeed)
                    glColor3d(1.0,0.12,0.08);
                else glColor3d(0.1,0.95,0.2);
                const auto end=sample.position+sample.normalDirection*toothLength;
                glVertex3d(sample.position.x,sample.position.y,sample.position.z);
                glVertex3d(end.x,end.y,end.z);
            }
            glEnd();
            drawList->AddText(
                {m_viewportRect.x+10.0f,m_viewportRect.y+10.0f},
                IM_COL32(225,225,225,255),
                std::format("Cluster |q'''| samples (log): red below feed, green at feed; peak {:.4g}",
                            maximum).c_str());
        }
        if(m_showExecutedJerkComb&&!m_executedJerkComb.empty()) {
            const auto toothLength=2.0*m_viewHalfHeight/height*20.0;
            glLineWidth(1.25f);
            glBegin(GL_LINES);
            for(const auto &tooth:m_executedJerkComb) {
                const auto used=std::clamp(tooth.utilization,0.0,1.0);
                const auto usedEnd=tooth.position+tooth.normal*(toothLength*used);
                const auto limitEnd=tooth.position+tooth.normal*toothLength;
                if(used>0.0) {
                    glColor4d(0.1,0.95,0.2,0.72);
                    glVertex3dv(glm::value_ptr(tooth.position));
                    glVertex3dv(glm::value_ptr(usedEnd));
                }
                if(used<1.0) {
                    glColor4d(1.0,0.12,0.08,0.65);
                    glVertex3dv(glm::value_ptr(usedEnd));
                    glVertex3dv(glm::value_ptr(limitEnd));
                }
            }
            glEnd();
        }
        glLineWidth(1.0f);
        drawPreview(PreviewBatch::G64ControlPolygon, m_previewRenderCache.g64ControlPolygon,
                    GL_LINES, 0.95f, 0.65f, 1.0f, 2, 0x3333);
        glPointSize(6.0f);
        drawPreview(PreviewBatch::G64ControlPoints, m_previewRenderCache.g64ControlPoints,
                    GL_POINTS, 1.0f, 0.75f, 1.0f);
        glPointSize(3.0f);
        glLineWidth(2.0f);
        drawPreview(PreviewBatch::G53Rapid, m_previewRenderCache.g53RapidLines,
                    GL_LINES, 1.0f, 1.0f, 0.4f, 10, 0x5555);
        drawPreview(PreviewBatch::Rapid, m_previewRenderCache.rapidLines,
                    GL_LINES, 0.4f, 0.4f, 1.0f, 4, 0x5555);
        glEnable(GL_POINT_SMOOTH);
        glPointSize(3.0f);
        drawPreview(PreviewBatch::DarkPoints, m_previewRenderCache.darkPoints,
                    GL_POINTS, 0.0f, 0.0f, 0.0f);
        drawPreview(PreviewBatch::LightPoints, m_previewRenderCache.lightPoints,
                    GL_POINTS, 0.1f, 0.55f, 0.1f);
        glDisable(GL_LINE_SMOOTH);
        glDisable(GL_POINT_SMOOTH);
        glLineWidth(1.0f);

        const auto simulation = m_simulation.snapshot();
        if(simulation.toolPose.geometry.number == 0)
            drawNoToolCrosshair(simulation.machinePosition);
        else if(simulation.status != ngc::SimulationStatus::Stopped)
            drawToolWireframe(simulation.toolPose);
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
        if(ImGui::Button("Clear Preview")) m_worker.clearPreview();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!m_previewRenderCache.hasSceneGeometry);
        if(ImGui::Button("Fit Toolpath to View")) {
            const auto width=std::max(static_cast<int>(std::round(
                m_viewportRect.z-m_viewportRect.x)),1);
            const auto height=std::max(static_cast<int>(std::round(
                m_viewportRect.w-m_viewportRect.y)),1);
            fitToolpathToView(width,height);
        }
        ImGui::EndDisabled();
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Fit the complete preview toolpath to the viewport (F).");
        ImGui::SameLine();
        ImGui::Checkbox("Cluster jerk comb", &m_showClusterGeometricJerkComb);
        if(ImGui::IsItemHovered())
            ImGui::SetTooltip("Log-scaled |q'''| teeth on short-entity cluster splines.\n"
                              "65 arc-length samples per planner piece. Red means the geometric\n"
                              "jerk speed cap is below programmed feed; green means it is not.");
        ImGui::SameLine();
        ImGui::Checkbox("Executed jerk comb",&m_showExecutedJerkComb);
        if(ImGui::IsItemHovered())
            ImGui::SetTooltip("One tooth every 10 executed servo periods at the tool tip.\n"
                              "Green is executed cubic path jerk; red is unused capacity.");

        const auto simulationActive = simulation.status == ngc::SimulationStatus::Running
            || simulation.status == ngc::SimulationStatus::Paused;
        ImGui::SameLine();
        ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
        ImGui::SameLine();
        ImGui::BeginDisabled(!compiledMode || m_programSource.empty() || simulationActive);
        if(ImGui::Button("Simulate")) {
            m_simulation.setTickMultiplier(m_simulationTickMultiplier);
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
        ImGui::BeginDisabled(simulationActive || !m_simulation.homingAvailable());
        if(ImGui::Button("Home")) {
            m_simulation.setTickMultiplier(m_simulationTickMultiplier);
            m_simulation.home();
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        if(ImGui::Button(m_showJogPane ? "Hide Jog" : "Jog")) m_showJogPane = !m_showJogPane;

        ImGui::SameLine();
        ImGui::BeginDisabled(simulationActive);
        if(ImGui::Button("Reset Simulation")) m_simulation.resetSimulation();
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        if(ImGui::InputInt("Speed multiplier", &m_simulationTickMultiplier, 1, 10)) {
            m_simulationTickMultiplier = std::clamp(m_simulationTickMultiplier, 1, 1000);
            m_simulation.setTickMultiplier(m_simulationTickMultiplier);
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

        m_simulation.setTickMultiplier(m_simulationTickMultiplier);
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
        if(ImGui::BeginChild("##mdi_history", { 0.0f, -inputHeight }, ImGuiChildFlags_Borders)) {
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

    static const char *axisLabel(const ngc::Machine::Axis axis) {
        switch(axis) {
            case ngc::Machine::Axis::X: return "X";
            case ngc::Machine::Axis::Y: return "Y";
            case ngc::Machine::Axis::Z: return "Z";
            case ngc::Machine::Axis::A: return "A";
            case ngc::Machine::Axis::B: return "B";
            case ngc::Machine::Axis::C: return "C";
        }
        return "?";
    }

    static ngc::AxisId jogAxis(const ngc::Machine::Axis axis) {
        return static_cast<ngc::AxisId>(static_cast<std::uint8_t>(axis));
    }

    static double axisValue(const ngc::position_t &position, const ngc::Machine::Axis axis) {
        switch(axis) {
            case ngc::Machine::Axis::X: return position.x;
            case ngc::Machine::Axis::Y: return position.y;
            case ngc::Machine::Axis::Z: return position.z;
            case ngc::Machine::Axis::A: return position.a;
            case ngc::Machine::Axis::B: return position.b;
            case ngc::Machine::Axis::C: return position.c;
        }
        return 0.0;
    }

    void renderJogPane(const ImGuiViewport &viewport, const ngc::SimulationSnapshot &simulation,
                       const float contentBottom) {
        const auto left = viewport.Pos.x + viewport.Size.x - m_jogPaneWidth;
        ImGui::SetNextWindowPos({ left, viewport.Pos.y + m_toolbarHeight });
        ImGui::SetNextWindowSize({ m_jogPaneWidth, contentBottom - viewport.Pos.y - m_toolbarHeight });
        constexpr auto flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
        ImGui::Begin("##jog_pane", nullptr, flags);

        ImGui::TextUnformatted("Jog");
        ImGui::Separator();
        const char *modes[] = { "Axes", "Coupled joints", "Individual joints" };
        ImGui::SetNextItemWidth(-FLT_MIN);
        if(ImGui::Combo("##jog_target_mode", &m_jogTargetMode, modes, 3)) {
            m_individualJogEnabled = false;
        }
        if(ImGui::RadioButton("Continuous", m_continuousJog)) m_continuousJog = true;
        ImGui::SameLine();
        if(ImGui::RadioButton("Incremental", !m_continuousJog)) m_continuousJog = false;

        auto speed = static_cast<float>(m_jogSpeedPercent);
        if(ImGui::SliderFloat("Speed", &speed, 1.0f, 100.0f, "%.0f%%")) m_jogSpeedPercent = speed;
        const std::array inchSteps { 0.001, 0.01, 0.1, 1.0 };
        const std::array millimetreSteps { 0.01, 0.1, 1.0, 10.0 };
        const auto &steps = m_machineUnit == ngc::Machine::Unit::Inch ? inchSteps : millimetreSteps;
        if(!m_continuousJog) {
            const auto preview = std::format("{:.3f}", steps[static_cast<std::size_t>(m_jogStepIndex)]);
            if(ImGui::BeginCombo("Step", preview.c_str())) {
                for(int index = 0; index < static_cast<int>(steps.size()); ++index) {
                    if(ImGui::Selectable(std::format("{:.3f}", steps[index]).c_str(), index == m_jogStepIndex))
                        m_jogStepIndex = index;
                }
                ImGui::EndCombo();
            }
        }

        if(m_jogTargetMode == 2) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.65f, 0.2f, 1.0f));
            ImGui::Checkbox("Enable individual joints", &m_individualJogEnabled);
            ImGui::PopStyleColor();
            ImGui::TextWrapped("Independent movement can rack a multi-motor gantry.");
        }
        ImGui::Separator();

        const auto otherMotion = (simulation.status == ngc::SimulationStatus::Running
                                  || simulation.status == ngc::SimulationStatus::Paused)
            && !simulation.jogging;
        bool heldThisFrame = false;
        const auto leaseTicks = static_cast<std::uint32_t>(std::max(
            1.0, std::ceil(0.020 / m_simulationTiming.servoPeriod)
                * static_cast<double>(m_simulationTickMultiplier)));
        const auto submitDirection = [&](const std::string &label, const ngc::JogTarget target,
                                         const ngc::JogMotionLimits limits,
                                         const ngc::JogMotionLimits stopLimits,
                                         const ngc::JogTravelRange travel, const double direction,
                                         const bool enabled) {
            ImGui::BeginDisabled(!enabled || otherMotion || (simulation.jogging && !m_uiContinuousJog));
            const auto clicked = ImGui::Button(label.c_str(), { 54.0f, 0.0f });
            const auto held = ImGui::IsItemActive();
            const auto activated = ImGui::IsItemActivated();
            ImGui::EndDisabled();
            if(m_continuousJog && held) {
                heldThisFrame = true;
                if(!m_uiContinuousJog && activated) {
                    const auto jog = m_nextJogId++;
                    const ngc::StartContinuousJogRequest request {
                        .id = m_nextJogRequest++, .jog = jog, .target = target,
                        .signedVelocity = direction * limits.velocity * m_jogSpeedPercent / 100.0,
                        .limits = limits, .stopLimits = stopLimits,
                        .travel = travel, .leaseTicks = leaseTicks,
                    };
                    if(m_simulation.startJog(ngc::ControlRequest { request })) {
                        m_uiContinuousJog = jog;
                        m_lastJogRenewal = m_time;
                    }
                } else if(m_time - m_lastJogRenewal >= 0.005) {
                    if(m_simulation.renewJog(m_nextJogRequest++, *m_uiContinuousJog))
                        m_lastJogRenewal = m_time;
                }
            } else if(!m_continuousJog && clicked && enabled && !otherMotion && !simulation.jogging) {
                const ngc::StartIncrementalJogRequest request {
                    .id = m_nextJogRequest++, .jog = m_nextJogId++, .target = target,
                    .distance = direction * steps[static_cast<std::size_t>(m_jogStepIndex)],
                    .velocity = limits.velocity * m_jogSpeedPercent / 100.0,
                    .limits = limits, .stopLimits = stopLimits, .travel = travel,
                };
                (void)m_simulation.startJog(ngc::ControlRequest { request });
            }
        };

        if(m_jogTargetMode < 2) {
            for(const auto &axis : m_axes) {
                ngc::JointMask joints = 0;
                auto velocity = axis.maxVelocity;
                auto acceleration = axis.maxAcceleration;
                auto jerk = std::numeric_limits<double>::infinity();
                for(const auto id : axis.joints) {
                    joints |= ngc::JointMask { 1 } << id;
                    const auto found = std::ranges::find(m_joints, id, &ngc::JointConfiguration::id);
                    if(found != m_joints.end()) {
                        velocity = std::min(velocity, found->maxVelocity);
                        acceleration = std::min(acceleration, found->maxAcceleration);
                        jerk = std::min(jerk, found->maxJerk);
                    }
                }
                const auto homed = (simulation.homedJoints & joints) == joints;
                const auto enabled = m_jogTargetMode == 1 || homed;
                ImGui::Text("%s  % .4f%s", axisLabel(axis.axis), axisValue(simulation.machinePosition, axis.axis),
                            homed ? "" : "  unhomed");
                ImGui::SameLine(std::max(100.0f, ImGui::GetWindowWidth() - 130.0f));
                const ngc::JogTarget target { ngc::JogTargetType::JointGroup, jogAxis(axis.axis), joints };
                const ngc::JogMotionLimits stopLimits { velocity, acceleration, jerk };
                const ngc::JogMotionLimits limits {
                    velocity,
                    std::min(acceleration, m_joggingConfiguration.acceleration),
                    std::min(jerk, m_joggingConfiguration.jerk),
                };
                const ngc::JogTravelRange travel { axis.minimum, axis.maximum, homed };
                submitDirection(std::format("-##{}", axisLabel(axis.axis)), target, limits, stopLimits,
                                travel, -1.0, enabled);
                ImGui::SameLine();
                submitDirection(std::format("+##{}", axisLabel(axis.axis)), target, limits, stopLimits,
                                travel, 1.0, enabled);
            }
        } else {
            for(const auto &joint : m_joints) {
                const auto mask = static_cast<ngc::JointMask>(ngc::JointMask { 1 } << joint.id);
                const auto homed = (simulation.homedJoints & mask) != 0;
                ImGui::Text("%s  % .4f%s", joint.name.c_str(), simulation.joints.position[joint.id],
                            homed ? "" : "  unhomed");
                ImGui::SameLine(std::max(100.0f, ImGui::GetWindowWidth() - 130.0f));
                const ngc::JogTarget target { ngc::JogTargetType::Joint, jogAxis(joint.axis), mask };
                const auto velocity = std::max(1e-6, joint.maxVelocity * 0.25);
                const ngc::JogMotionLimits stopLimits {
                    velocity, joint.maxAcceleration, joint.maxJerk };
                const ngc::JogMotionLimits limits {
                    velocity,
                    std::min(joint.maxAcceleration, m_joggingConfiguration.acceleration),
                    std::min(joint.maxJerk, m_joggingConfiguration.jerk),
                };
                const auto low = std::min(joint.minimum * joint.coordinateScale,
                                          joint.maximum * joint.coordinateScale);
                const auto high = std::max(joint.minimum * joint.coordinateScale,
                                           joint.maximum * joint.coordinateScale);
                const ngc::JogTravelRange travel { low, high, homed };
                submitDirection(std::format("-##joint{}", joint.id), target, limits, stopLimits, travel, -1.0,
                                m_individualJogEnabled);
                ImGui::SameLine();
                submitDirection(std::format("+##joint{}", joint.id), target, limits, stopLimits, travel, 1.0,
                                m_individualJogEnabled);
            }
        }

        if(m_uiContinuousJog && (!heldThisFrame || !m_continuousJog || !m_showJogPane)) {
            (void)m_simulation.stopJog(m_nextJogRequest++, *m_uiContinuousJog);
            m_uiContinuousJog.reset();
        }

        ImGui::Separator();
        if(simulation.lastJogStopReason) {
            const auto reason = [&] {
                switch(*simulation.lastJogStopReason) {
                    case ngc::JogStopReason::TargetReached: return "target reached";
                    case ngc::JogStopReason::RequestedStop: return "released";
                    case ngc::JogStopReason::LeaseExpired: return "lease expired";
                    case ngc::JogStopReason::LimitReached: return "limit reached";
                    case ngc::JogStopReason::Disabled: return "disabled";
                    case ngc::JogStopReason::Aborted: return "aborted";
                    case ngc::JogStopReason::Fault: return "fault";
                    case ngc::JogStopReason::Superseded: return "superseded";
                }
                return "unknown";
            }();
            ImGui::TextDisabled("Last stop: %s", reason);
        }
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.08f, 0.08f, 1.0f));
        if(ImGui::Button("STOP MOTION", { -FLT_MIN, 0.0f })) {
            if(m_uiContinuousJog) {
                (void)m_simulation.stopJog(m_nextJogRequest++, *m_uiContinuousJog);
                m_uiContinuousJog.reset();
            } else {
                m_simulation.stop();
            }
        }
        ImGui::PopStyleColor();
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
        auto modalGCodes = m_worker.lock([&] { return m_worker.machine().activeModalGCodes(); });
        const auto simulationHasModalState = simulation.status != ngc::SimulationStatus::Stopped
            && !simulation.activeModalGCodes.empty();
        if(simulationHasModalState) modalGCodes = simulation.activeModalGCodes;
        std::string modalText;
        for(const auto &code : modalGCodes) {
            if(!modalText.empty()) modalText += ' ';
            modalText += code;
        }

        auto diagnosticText=std::format("Simulation: {}    MCS XYZ: {:.4f}, {:.4f}, {:.4f}",
            statusText,simulation.machinePosition.x,simulation.machinePosition.y,
            simulation.machinePosition.z);
        if(simulation.toolPose.geometry.number!=0)
            diagnosticText+=std::format("    Tool XYZ: {:.4f}, {:.4f}, {:.4f}",
                simulation.toolPosition.x,simulation.toolPosition.y,simulation.toolPosition.z);
        if(simulation.status != ngc::SimulationStatus::Stopped) {
            const auto elapsed = std::max(simulation.programElapsedSeconds, 0.0);
            const auto totalWholeSeconds = static_cast<std::uint64_t>(elapsed);
            const auto hours = totalWholeSeconds / 3600;
            const auto minutes = totalWholeSeconds / 60 % 60;
            const auto seconds = elapsed - static_cast<double>(hours * 3600 + minutes * 60);
            diagnosticText+=std::format(
                "\nG-code elapsed {:.3f} s ({:02}:{:02}:{:06.3f})    Servo {:.3f} ms    "
                "Scheduler {:.3f} ms ({} ticks) x{}    Ticks {}    Missed deadlines {}    "
                "Max wake {:.1f} us    Max tick {:.1f} us",
                elapsed,hours,minutes,seconds,simulation.servoPeriodSeconds*1000.0,
                simulation.schedulerPeriodSeconds*1000.0,
                simulation.servoTicksPerSchedulerPeriod, simulation.tickMultiplier,
                simulation.servoTicks,simulation.deadlineMisses,
                simulation.maximumWakeLatenessSeconds*1.0e6,
                simulation.maximumTickExecutionSeconds * 1.0e6);
            if(!simulation.trajectoryPlanningActivity.empty())
                diagnosticText+=std::format("\nPlanning {:.3f} s: {}",
                    simulation.trajectoryPlanningActivitySeconds,
                    simulation.trajectoryPlanningActivity);
            if(!simulation.trajectoryDriverActivity.empty())
                diagnosticText+="\nTrajectory driver: "+simulation.trajectoryDriverActivity;
            if(!simulation.trajectoryContinuousPlanSummary.empty())
                diagnosticText+="\nContinuous plan: "+simulation.trajectoryContinuousPlanSummary;
            const auto backendState=[](const ngc::BackendState state) {
                switch(state) {
                    case ngc::BackendState::Disabled: return "Disabled";
                    case ngc::BackendState::Held: return "Held";
                    case ngc::BackendState::Running: return "Running";
                    case ngc::BackendState::Faulted: return "Faulted";
                }
                return "Unknown";
            };
            diagnosticText+=std::format(
                "\nTrajectory backend: {} epoch={} chunk={} span={} progress={:.6f} "
                "velocity={:.6g} acceleration={:.6g} branch={} fault={}\n"
                "Committed normal motion: {:.6f} s (active {:.6f} s + queued {:.6f} s, "
                "{} items)    Stop branch: {:.6f} s",
                backendState(simulation.trajectoryBackendState),
                simulation.trajectoryBackendEpoch,simulation.trajectoryBackendChunk,
                simulation.trajectoryBackendSpan,simulation.trajectoryBackendSpanProgress,
                simulation.trajectoryBackendVelocity,
                simulation.trajectoryBackendAcceleration,
                simulation.trajectoryBackendLastBranch,simulation.trajectoryBackendFaultCode,
                simulation.trajectoryBackendCommittedNormalSeconds,
                simulation.trajectoryBackendActiveNormalRemainingSeconds,
                simulation.trajectoryBackendQueuedNormalSeconds,
                simulation.trajectoryBackendQueuedExecutionItems,
                simulation.trajectoryBackendStopBranchSeconds);
            if(!simulation.trajectoryBackendSpanDetail.empty())
                diagnosticText+="\nActive execution span: "+simulation.trajectoryBackendSpanDetail;
        }
        if(!modalText.empty()) diagnosticText+="\nModal: "+modalText;
        const auto diagnosticLines=1+std::ranges::count(diagnosticText,'\n');
        const auto diagnosticHeight=ImGui::GetTextLineHeightWithSpacing()
            *static_cast<float>(diagnosticLines)+2.0f*ImGui::GetStyle().FramePadding.y;
        ImGui::InputTextMultiline("##simulation_diagnostics",diagnosticText.data(),
            diagnosticText.size()+1,ImVec2(-1.0f,diagnosticHeight),
            ImGuiInputTextFlags_ReadOnly);
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
                                            viewport.Size.x - (m_showJogPane ? m_jogPaneWidth : 0.0f) - 240.0f);
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
            viewport.Pos.x + viewport.Size.x - (m_showJogPane ? m_jogPaneWidth : 0.0f),
            contentBottom,
        };

        renderToolbar(viewport, simulation);
        renderProgramPane(viewport, simulation, contentBottom);
        if(m_showJogPane) renderJogPane(viewport, simulation, contentBottom);
        else if(m_uiContinuousJog) {
            (void)m_simulation.stopJog(m_nextJogRequest++, *m_uiContinuousJog);
            m_uiContinuousJog.reset();
        }
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
            if(ImGui::BeginChild("top", { 0, 0 }, ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY)) {
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
                    m_simulation.setTickMultiplier(m_simulationTickMultiplier);
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
                if(ImGui::InputInt("Speed multiplier", &m_simulationTickMultiplier, 1, 10)) {
                    m_simulationTickMultiplier = std::clamp(m_simulationTickMultiplier, 1, 1000);
                    m_simulation.setTickMultiplier(m_simulationTickMultiplier);
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
                ImGui::Text("Simulation: %s  MCS XYZ: %.4f, %.4f, %.4f",
                            statusText, simulation.machinePosition.x, simulation.machinePosition.y,
                            simulation.machinePosition.z);
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

            if(ImGui::BeginChild("middle", {0, 0 }, ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY)) {
                for(const auto &error : m_worker.parserErrors()) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4_Redish);
                    ImGui::Selectable(error.text().c_str());
                    ImGui::PopStyleColor();
                }
            }

            ImGui::EndChild();

            if(ImGui::BeginChild("bottom", {0, 0 }, ImGuiChildFlags_Borders)) {
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

Application::Application(GLFWwindow *window, const ngc::MachineConfiguration &configuration)
    : m_impl(std::make_unique<ApplicationImpl>(window, configuration)) { }
Application::~Application() = default;

void Application::init() { m_impl->init(); }
void Application::preRender() { m_impl->preRender(); }
void Application::setLastFrameRenderSeconds(const double seconds) {
    m_impl->setLastFrameRenderSeconds(seconds);
}
void Application::addScroll(const double delta) { m_impl->addScroll(delta); }
void Application::terminate() { m_impl->terminate(); }
void Application::render() { m_impl->render(); }
void Application::render3D() { m_impl->render3D(); }
