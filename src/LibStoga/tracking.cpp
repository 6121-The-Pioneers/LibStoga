#include "tracking.h"
#include "error.h"

ls::TrackingWheel::TrackingWheel(std::uint8_t port, double radius, bool reversed)
{
    if (port < 1 || port > 21) {
        THROW_ERROR("Port must be in the range of [1, 21].");
    }
    rotation = std::make_unique<pros::Rotation>(port);
    if (reversed) rotation->reverse();
    rotation->set_data_rate(5);
    encoder = nullptr;
    setRadius(radius);
}

ls::TrackingWheel::TrackingWheel(std::uint8_t portUpper, std::uint8_t portLower, double radius, bool Reversed)
{
    if (portUpper < 'A' || portUpper > 'H') {
        THROW_ERROR("Upper port must be in the range of [A-H]");
    } else if (portLower < 'A' || portLower > 'H') {
        THROW_ERROR("Lower port must be in the range of [A-H]");
    }
    port_upper = portUpper;
    port_lower = portLower;
    reversed = Reversed;
    encoder = std::make_unique<pros::adi::Encoder>(port_upper, port_lower, reversed);
    rotation = nullptr;
    setRadius(radius);
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
        conversion_factor = (6.28318531 * radius) / 36000.0;
    }
    else {
        conversion_factor = (6.28318531 * radius) / 360.0;
    }
}

void ls::TrackingWheel::reset()
{
    if (encoder == nullptr) {
        rotation->reset_position();
    } else {
        encoder->reset();
    }
}
