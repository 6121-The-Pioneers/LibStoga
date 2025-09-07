#pragma once
#include <vector>
#include <random>
#include "geometry.h"
#include "odom.h"

namespace ls {
    struct Particle { double x; double y; double heading; double weight; };

    class MCLTracker : public AbstractOdom {
    public:
        struct Config { size_t count = 200; double processNoiseXY = 0.5; double processNoiseTheta = 2; double fieldW = 144; double fieldH = 144; };
        explicit MCLTracker(const Config& cfg): config(cfg) { initParticles(); }
        void initialize(std::initializer_list<uint8_t>) override {} // not used directly
        void compute() override; // propagate + resample -> update pos
        double getDeltaX() override { return deltaX; }
        double getDeltaY() override { return deltaY; }
        Angle getDeltaAngle() override { return Angle(deltaTheta); }
        void injectMeasurement(double x, double y, double headingDeg, double stdXY = 2, double stdHeading = 5);
    private:
        Config config;
        std::vector<Particle> particles;
        std::default_random_engine rng{12345};
        double deltaX = 0, deltaY = 0, deltaTheta = 0; // since last compute
        void initParticles();
        void motionUpdate();
        void normalize();
        void resample();
        void updateEstimate();
    };
}
