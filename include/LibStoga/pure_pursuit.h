#include <array>
#include "LibStoga/spid.h"

namespace ls {
    class purePursuit {
    public:
        purePursuit(std::array<double, 10> targx, std::array<double, 10> targy); //constructor        
        void move();//moves the bot
    protected:
        double x;//bot x position
        double y;//bot y position
        double spd;//bot speed
        double hdg;//bot heading
        int wp;//current waypoint target, indexer for target arrays
        double dist;//distance between bot and waypoint

        SmartPID spidMove;//spid values for moving the bot
        SmartPID spidTurn;//spid values for turning the bot

        std::array<double, 10> targx;//the x coords of all the target waypoints
        std::array<double, 10> targy;//the y coords of aa the target waypoints

        double rx;//the x position of the current target waypoint reletive to the bot
        double ry;//the y position of the current target waypoint reletive to the bot
        double rh;//the direction the bot needs to be facing to be moving strait towards the target
    }; 
}
