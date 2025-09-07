#pragma once
#include "geometry.h"
#include "odom.h"
#include <optional>

namespace ls {
    struct PursuitOutput {
        double linear{0};
        double angular{0};
        bool done{false};
    };

    class PurePursuit {
    public:
        explicit PurePursuit(AbstractOdom& tracker, double lookahead, double kTurn = 1.0): odom(tracker), lookaheadDist(lookahead), kTurn(kTurn) {}
        void setPath(const Path& p) { path = p; currentIdx = 0; }
        void setLookahead(double l) { lookaheadDist = l; }
        PursuitOutput update();
        bool isFinished() const { return finished; }
    private:
        AbstractOdom& odom;
        Path path;
        size_t currentIdx{0};
        double lookaheadDist; // inches
        double kTurn; // curvature gain
        bool finished{false};
        std::optional<Point> findLookahead(const Pose& robot);
    };
}
