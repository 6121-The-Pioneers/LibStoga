#ifndef LS_CHASSIS_H
#define LS_CHASSIS_H

#include "api.h"
#include "odom.h"

namespace ls {
    class Chassis
    {
    private:
        pros::MotorGroup* right;
        pros::MotorGroup* left;
        ls::AbstractOdom* odom;

    public:
        Chassis(pros::MotorGroup* right, pros::MotorGroup* left, ls::AbstractOdom* odom);

        void moveTo();
        
        ~Chassis() {};
    };    
}



#endif // LS_CHASSIS_H