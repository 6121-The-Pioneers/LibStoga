#include "LibStoga/pure_pursuit.h"
#include "LibStoga/spid.h"

namespace ls {



    void purePursuit::move() {
        targx = xcoords[wp];
        targy = ycoords[wp];
        rx = targx - x;
        ry = targy - y;
        rh = degrees(atan2(rx,ry));
        // xv = spd*sin(hdg*(PI/180));
        // yv = spd*cos(hdg*(PI/180));
        dist = math.sqrt(math.pow(rx, 2)+math.pow(ry, 2));
        
        if(hdg > 360) {
            hdg -= 360;
        } else if (hdg<0) {
            hdg += 360;
        }
        if(rh<0) {
            rh += 360;
        } else if(rh>360) {
            rh -= 360;
        }
        if(abs(targx-x) <= 1 && abs(targy-y) <= 1 && wp >= xcoords.length-1) {
            spd = 0;
        } else if(abs(targx-x) <= 30 && abs(targy-y) <= 30 && wp < xcoords.length-1) {
            wp ++;
        }
        // x+=xv;
        // y+=yv;
}


}
