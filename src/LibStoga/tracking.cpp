#include "tracking.h"
#include <stdexcept>

ls::TrackingWheel::TrackingWheel(std::uint8_t port, double radius, bool reversed)
{
    if (port < 0 || port > 24) {
        std::invalid_argument("Port must be in the range of [0, 24].");
    }
    rotation = std::make_unique<pros::Rotation>(port);
    if (reversed) rotation->reverse();
    rotation->set_data_rate(5);
    encoder = nullptr;
    setRadius(radius);
}

ls::TrackingWheel::TrackingWheel(std::uint8_t portUpper, std::uint8_t portLower, double radius, bool Reversed)
{
    port_upper = portUpper; // how do you check these?
    port_lower = portLower; // how do you check these?
    reversed = Reversed;
    encoder = std::make_unique<pros::adi::Encoder>(port_upper, port_lower, reversed);
    rotation = nullptr;
    setRadius(radius);
}

ls::TrackingWheel::TrackingWheel(TrackingWheel &other)
{
    if (other.encoder == nullptr) {
        other.rotation.swap(rotation);
    } else {
        other.encoder.swap(encoder);
        port_upper = other.port_upper;
        port_lower = other.port_lower;
        reversed = other.reversed;
        prev_distance = other.prev_distance;
        prev_time = other.prev_time;
    }
    radius = other.radius;
    conversion_factor = other.conversion_factor;

    other.port_upper = 0;
    other.port_lower = 0;
    other.reversed = 0;
    other.prev_distance = 0;
    other.prev_time = 0;
    other.radius = 0;
    other.conversion_factor = 0;
}

void ls::TrackingWheel::reverse()
{
    if (encoder == nullptr) {
        rotation->reverse();
    } else {
        reversed = !reversed;
        encoder.reset(new pros::adi::Encoder(port_upper, port_lower, reversed));
    }
}

double ls::TrackingWheel::getLinearSpeed()
{
    if (encoder == nullptr) {
        return rotation->get_velocity() * conversion_factor;
    } else {
        // Having to calculate change in distance per time manually.
        long double current_time = pros::millis() / 1000.0;
        const long double dt = current_time - prev_time;
        prev_time = current_time;

        const double tor = encoder->get_value() - prev_distance;
        prev_distance = encoder->get_value();
        
        return (tor * conversion_factor) / dt;
    }
}

double ls::TrackingWheel::getLinearDistance()
{
    if (encoder == nullptr) {
        return rotation->get_position() * conversion_factor;
    } else {
        return encoder->get_value() * conversion_factor;
    }
}

double ls::TrackingWheel::getLinearDeltaDistance()
{
    const double latest = getLinearDistance();
    double tor = latest - prev_distance;
    prev_distance = latest;
    return tor;
}

void ls::TrackingWheel::setRadius(double wr)
{
    radius = wr;
    if (encoder == nullptr) {
        conversion_factor = radius / 360000.0;
    }
    else {
        conversion_factor = radius / 360.0;
    }
}