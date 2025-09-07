#include "pure_pursuit.h"
#include <cmath>

namespace ls {
    std::optional<Point> PurePursuit::findLookahead(const Pose &robot) {
        // simple linear scan from currentIdx; choose first point at >= lookahead distance
        for (size_t i = currentIdx; i < path.size(); ++i) {
            if (distance(robot, path[i]) >= lookaheadDist) {
                currentIdx = i; // advance index
                return path[i];
            }
        }
        return std::nullopt;
    }

    PursuitOutput PurePursuit::update() {
        PursuitOutput out;
        if (path.empty()) { out.done = true; finished = true; return out; }
        Pose robot(odom.getX(), odom.getY(), odom.getAngle());
        // if near final point mark done
        if (distance(robot, path.back()) < lookaheadDist * 0.5) {
            out.done = true; finished = true; return out; }

        auto look = findLookahead(robot);
        if (!look) { out.done = true; finished = true; return out; }

        // Compute curvature to lookahead point (field coordinates -> convert to robot frame)
        double headingRad = degreesToRadians(90 - robot.heading.getAngle()); // convert bearing to math frame
        double dx = look->x - robot.x;
        double dy = look->y - robot.y;
        // rotate into robot frame
        double rx =  dx * std::cos(headingRad) + dy * std::sin(headingRad);
        double ry = -dx * std::sin(headingRad) + dy * std::cos(headingRad);
        if (rx <= 1e-6) { rx = 1e-6; }
        double curvature = (2 * ry) / (lookaheadDist * lookaheadDist);
        curvature = std::clamp(curvature, -1.0, 1.0); // cap extreme turns
        out.linear = std::clamp( (lookaheadDist * 8.0), 40.0, 110.0); // heuristic scaling
        out.angular = out.linear * curvature * kTurn;
        return out;
    }
}
