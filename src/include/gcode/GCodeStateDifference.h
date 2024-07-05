#pragma once

#include "gcode/GCode.h"
#include <optional>
#include <utility>

namespace ngc {
    struct GCodeMovement {
        std::optional<double> X, Y, Z, A, B, C, I, J, K;

        bool empty() const {
            return !(X || Y || Z || A || B || C || I || J || K);
        }
    };

    class GCodeStateDifference {
        std::optional<GCMotion> m_modeMotion;
        std::optional<GCPlane> m_modePlane;
        std::optional<GCDist> m_modeDistance;
        std::optional<GCFeed> m_modeFeedrate;
        std::optional<GCUnits> m_modeUnits;
        std::optional<GCTLen> m_modeToolOffset;
        std::optional<GCCoord> m_modeCoordSys;
        std::optional<GCPath> m_modePath;

        std::optional<GCNonModal> m_nonModal;

        std::optional<MCSpindle> m_modeSpindle;
        std::optional<MCStop> m_modeStop;
        std::optional<MCToolChange> m_modeToolChange;

        std::optional<double> m_A, m_B, m_C, m_D, m_F, m_H, m_I, m_J, m_K, m_L, m_P, m_Q, m_R, m_S, m_T, m_X, m_Y, m_Z;

    public:
        GCodeStateDifference(const GCodeState &a, const GCodeState &b) {
            if(a.modeMotion() != b.modeMotion()) { m_modeMotion = b.modeMotion(); }
            if(a.modePlane() != b.modePlane()) { m_modePlane = b.modePlane(); }
            if(a.modeDistance() != b.modeDistance()) { m_modeDistance = b.modeDistance(); }
            if(a.modeFeedrate() != b.modeFeedrate()) { m_modeFeedrate = b.modeFeedrate(); }
            if(a.modeUnits() != b.modeUnits()) { m_modeUnits = b.modeUnits(); }
            if(a.modeToolOffset() != b.modeToolOffset()) { m_modeToolOffset = b.modeToolOffset(); }
            if(a.modeCoordSys() != b.modeCoordSys()) { m_modeCoordSys = b.modeCoordSys(); }
            if(a.modePath() != b.modePath()) { m_modePath = b.modePath(); }

            if(a.nonModal() != b.nonModal()) { m_nonModal = b.nonModal(); }

            if(a.modeSpindle() != b.modeSpindle()) { m_modeSpindle = b.modeSpindle(); }
            if(a.modeStop() != b.modeStop()) { m_modeStop = b.modeStop(); }
            if(a.modeToolChange() != b.modeToolChange()) { m_modeToolChange = b.modeToolChange(); }

            if(a.A() != b.A()) { m_A = b.A(); }
            if(a.B() != b.B()) { m_B = b.B(); }
            if(a.C() != b.C()) { m_C = b.C(); }
            if(a.D() != b.D()) { m_D = b.D(); }
            if(a.F() != b.F()) { m_F = b.F(); }
            if(a.H() != b.H()) { m_H = b.H(); }
            if(a.I() != b.I()) { m_I = b.I(); }
            if(a.J() != b.J()) { m_J = b.J(); }
            if(a.K() != b.K()) { m_K = b.K(); }
            if(a.L() != b.L()) { m_L = b.L(); }
            if(a.P() != b.P()) { m_P = b.P(); }
            if(a.Q() != b.Q()) { m_Q = b.Q(); }
            if(a.R() != b.R()) { m_R = b.R(); }
            if(a.S() != b.S()) { m_S = b.S(); }
            if(a.T() != b.T()) { m_T = b.T(); }
            if(a.X() != b.X()) { m_X = b.X(); }
            if(a.Y() != b.Y()) { m_Y = b.Y(); }
            if(a.Z() != b.Z()) { m_Z = b.Z(); }
        }

        std::optional<GCMotion> modeMotion() const { return m_modeMotion; }
        std::optional<GCPlane> modePlane() const { return m_modePlane; }
        std::optional<GCDist> modeDistance() const { return m_modeDistance; }
        std::optional<GCFeed> modeFeedrate() const { return m_modeFeedrate; }
        std::optional<GCUnits> modeUnits() const { return m_modeUnits; }
        std::optional<GCTLen> modeToolLengthOffset() const { return m_modeToolOffset; }
        std::optional<GCCoord> modeCoordSys() const { return m_modeCoordSys; }
        std::optional<GCPath> modePath() const { return m_modePath; }
        std::optional<MCSpindle> modeSpindle() const { return m_modeSpindle; }
        std::optional<MCStop> modeStop() const { return m_modeStop; }
        std::optional<MCToolChange> modeToolChange() const { return m_modeToolChange; }
        std::optional<GCNonModal> monModal() const { return m_nonModal; }

        std::optional<double> A() const { return m_A; }
        std::optional<double> B() const { return m_B; }
        std::optional<double> C() const { return m_C; }
        std::optional<double> D() const { return m_D; }
        std::optional<double> F() const { return m_F; }
        std::optional<double> H() const { return m_H; }
        std::optional<double> I() const { return m_I; }
        std::optional<double> J() const { return m_J; }
        std::optional<double> K() const { return m_K; }
        std::optional<double> L() const { return m_L; }
        std::optional<double> P() const { return m_P; }
        std::optional<double> Q() const { return m_Q; }
        std::optional<double> R() const { return m_R; }
        std::optional<double> S() const { return m_S; }
        std::optional<double> T() const { return m_T; }
        std::optional<double> X() const { return m_X; }
        std::optional<double> Y() const { return m_Y; }
        std::optional<double> Z() const { return m_Z; }

        std::optional<GCMotion> takeModeMotion() { return std::exchange(m_modeMotion, std::nullopt); }
        std::optional<GCPlane> takeModePlane() { return std::exchange(m_modePlane, std::nullopt); }
        std::optional<GCDist> takeModeDistance() { return std::exchange(m_modeDistance, std::nullopt); }
        std::optional<GCFeed> takeModeFeedrate() { return std::exchange(m_modeFeedrate, std::nullopt); }
        std::optional<GCUnits> takeModeUnits() { return std::exchange(m_modeUnits, std::nullopt); }
        std::optional<GCTLen> takeModeToolLengthOffset() { return std::exchange(m_modeToolOffset, std::nullopt); }
        std::optional<GCCoord> takeModeCoordSys() { return std::exchange(m_modeCoordSys, std::nullopt); }
        std::optional<GCPath> takeModePath() { return std::exchange(m_modePath, std::nullopt); }
        std::optional<MCSpindle> takeModeSpindle() { return std::exchange(m_modeSpindle, std::nullopt); }
        std::optional<MCStop> takeModeStop() { return std::exchange(m_modeStop, std::nullopt); }
        std::optional<MCToolChange> takeModeToolChange() { return std::exchange(m_modeToolChange, std::nullopt); }
        std::optional<GCNonModal> takeNonModal() { return std::exchange(m_nonModal, std::nullopt); }

        std::optional<double> takeA() { return std::exchange(m_A, std::nullopt); }
        std::optional<double> takeB() { return std::exchange(m_B, std::nullopt); }
        std::optional<double> takeC() { return std::exchange(m_C, std::nullopt); }
        std::optional<double> takeD() { return std::exchange(m_D, std::nullopt); }
        std::optional<double> takeF() { return std::exchange(m_F, std::nullopt); }
        std::optional<double> takeH() { return std::exchange(m_H, std::nullopt); }
        std::optional<double> takeI() { return std::exchange(m_I, std::nullopt); }
        std::optional<double> takeJ() { return std::exchange(m_J, std::nullopt); }
        std::optional<double> takeK() { return std::exchange(m_K, std::nullopt); }
        std::optional<double> takeL() { return std::exchange(m_L, std::nullopt); }
        std::optional<double> takeP() { return std::exchange(m_P, std::nullopt); }
        std::optional<double> takeQ() { return std::exchange(m_Q, std::nullopt); }
        std::optional<double> takeR() { return std::exchange(m_R, std::nullopt); }
        std::optional<double> takeS() { return std::exchange(m_S, std::nullopt); }
        std::optional<double> takeT() { return std::exchange(m_T, std::nullopt); }
        std::optional<double> takeX() { return std::exchange(m_X, std::nullopt); }
        std::optional<double> takeY() { return std::exchange(m_Y, std::nullopt); }
        std::optional<double> takeZ() { return std::exchange(m_Z, std::nullopt); }

        GCodeMovement takeMovement() {
            return { takeX(), takeY(), takeZ(), takeA(), takeB(), takeC(), takeI(), takeJ(), takeK() };
        }

        bool empty() const {
            if(m_modeMotion) { return false; }
            if(m_modePlane) { return false; }
            if(m_modeDistance) { return false; }
            if(m_modeFeedrate) { return false; }
            if(m_modeUnits) { return false; }
            if(m_modeToolOffset) { return false; }
            if(m_modeCoordSys) { return false; }
            if(m_modePath) { return false; }

            if(m_nonModal) { return false; }

            if(m_modeSpindle) { return false; }
            if(m_modeStop) { return false; }
            if(m_modeToolChange) { return false; }

            if(m_A || m_B || m_C || m_D || m_F || m_H || m_I || m_J || m_K || m_L || m_P || m_Q || m_R || m_S || m_T || m_X || m_Y || m_Z) { return false; }

            return true;
        }

        std::string text() const {
            std::string text = "GCodeStateDifference("; 

            if(m_modeMotion) { text += std::format("{}, ", name(*m_modeMotion)); }
            if(m_modePlane) { text += std::format("{}, ", name(*m_modePlane)); }
            if(m_modeDistance) { text += std::format("{}, ", name(*m_modeDistance)); }
            if(m_modeFeedrate) { text += std::format("{}, ", name(*m_modeFeedrate)); }
            if(m_modeUnits) { text += std::format("{}, ", name(*m_modeUnits)); }
            if(m_modeToolOffset) { text += std::format("{}, ", name(*m_modeToolOffset)); }
            if(m_modeCoordSys) { text += std::format("{}, ", name(*m_modeCoordSys)); }
            if(m_modePath) { text += std::format("{}, ", name(*m_modePath)); }
            if(m_nonModal) { text += std::format("{}, ", name(*m_nonModal)); }
            if(m_modeSpindle) { text += std::format("{}, ", name(*m_modeSpindle)); }
            if(m_modeStop) { text += std::format("{}, ", name(*m_modeStop)); }
            if(m_modeToolChange) { text += std::format("{}, ", name(*m_modeToolChange)); }

            if(m_A) { text += "A, "; }
            if(m_B) { text += "B, "; }
            if(m_C) { text += "C, "; }
            if(m_D) { text += "D, "; }
            if(m_F) { text += "F, "; }
            if(m_H) { text += "H, "; }
            if(m_I) { text += "I, "; }
            if(m_J) { text += "J, "; }
            if(m_K) { text += "K, "; }
            if(m_L) { text += "L, "; }
            if(m_P) { text += "P, "; }
            if(m_Q) { text += "Q, "; }
            if(m_R) { text += "R, "; }
            if(m_S) { text += "S, "; }
            if(m_T) { text += "T, "; }
            if(m_X) { text += "X, "; }
            if(m_Y) { text += "Y, "; }
            if(m_Z) { text += "Z, "; }

            text += ")";

            return text;
        }
    };
}