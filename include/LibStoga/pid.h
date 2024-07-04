/*
* Contains all PID related classes and methods.
*/
#ifndef PID_H
#define PID_H

namespace ls {
    /**
     * @brief The PID object, calculates PID outputs given an error.
     */
    class PID {
        public:
            /**
             * @brief Construct a new PID object
             * 
             * @param kP proportion constant
             * @param kI integral constant
             * @param kD derivative constant
             * @param windupRange desired windup range
             * @param signFlipReset if signed is fliped already.
             */
            PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);
            /**
             * @brief calculate the new PID value given the error.
             * 
             * @param error the error from target
             * @return the value from PID
             */
            float update(const float error);
            /**
             * @brief Resets the integral and other variables to reset PID state to original. 
             */
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
