#ifndef PID_H
#define PID_H

namespace LibStoga {
    class PID {
        public:
            PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);

            float update(const float error);
            void reset();

        protected:
            float kP;
            float kI;
            float kD;
            float windupRange;
            bool signFlipReset;
            float integral;
            float prevError;
        };
} 

#endif 
