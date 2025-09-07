#include "mcl.h"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace ls {
    void MCLTracker::initParticles() {
        std::uniform_real_distribution<double> dx(0, config.fieldW);
        std::uniform_real_distribution<double> dy(0, config.fieldH);
        std::uniform_real_distribution<double> dh(0, 360);
        particles.resize(config.count);
        for (auto &p : particles) { p.x = dx(rng); p.y = dy(rng); p.heading = dh(rng); p.weight = 1.0 / config.count; }
    }

    void MCLTracker::motionUpdate() {
        std::normal_distribution<double> nx(0, config.processNoiseXY);
        std::normal_distribution<double> ny(0, config.processNoiseXY);
        std::normal_distribution<double> nt(0, config.processNoiseTheta);
        // use odom deltas if a subordinate odom is desired (future)
        for (auto &p : particles) {
            p.x += nx(rng);
            p.y += ny(rng);
            p.heading += nt(rng);
            if (p.heading >= 360) p.heading -= 360; if (p.heading < 0) p.heading += 360;
        }
    }

    void MCLTracker::normalize() {
        double sum = 0; for (auto &p : particles) sum += p.weight; if (sum <= 0) { for (auto &p : particles) p.weight = 1.0 / particles.size(); return; }
        for (auto &p : particles) p.weight /= sum;
    }

    void MCLTracker::resample() {
        std::vector<Particle> newCloud(particles.size());
        std::vector<double> cdf(particles.size());
        double cumulative = 0;
        for (size_t i = 0; i < particles.size(); ++i) { cumulative += particles[i].weight; cdf[i] = cumulative; }
        std::uniform_real_distribution<double> dist(0,1);
        for (size_t i = 0; i < particles.size(); ++i) {
            double r = dist(rng);
            auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
            size_t idx = (it == cdf.end()) ? cdf.size() -1 : (it - cdf.begin());
            newCloud[i] = particles[idx]; newCloud[i].weight = 1.0 / particles.size();
        }
        particles.swap(newCloud);
    }

    void MCLTracker::updateEstimate() {
        double x=0,y=0; double sx=0, sy=0; // for circular mean of heading
        for (auto &p : particles) { x += p.x * p.weight; y += p.y * p.weight; double rad = p.heading * M_PI/180.0; sx += std::cos(rad)*p.weight; sy += std::sin(rad)*p.weight; }
        double avgHeading = std::atan2(sy, sx) * 180.0 / M_PI; if (avgHeading < 0) avgHeading += 360;
        deltaX = x - pos.X; deltaY = y - pos.Y; deltaTheta = angleDiff(avgHeading, pos.theta.getAngle());
        pos.X = x; pos.Y = y; pos.theta = avgHeading;
    }

    void MCLTracker::compute() {
        motionUpdate();
        normalize();
        resample();
        updateEstimate();
    }

    void MCLTracker::injectMeasurement(double mx, double my, double mh, double stdXY, double stdHeading) {
        std::normal_distribution<double> gx(0, stdXY); std::normal_distribution<double> gy(0, stdXY); std::normal_distribution<double> gh(0, stdHeading);
        // Weight particles by gaussian distance to measurement
        double varXY = stdXY*stdXY; double varH = stdHeading*stdHeading;
        double inv2piXY = 1.0/(2*M_PI*varXY); double inv2piH = 1.0/(std::sqrt(2*M_PI*varH));
        for (auto &p: particles) {
            double dx = p.x - mx; double dy = p.y - my; double d2 = dx*dx + dy*dy;
            double da = angleDiff(p.heading, mh);
            double w = std::exp(-d2/(2*varXY)) * std::exp(-(da*da)/(2*varH));
            p.weight = w + 1e-9;
        }
        normalize();
    }
}
