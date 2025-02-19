/**
 * @file pid.h
 * @author Asish Nakka
 * @brief Contains all PID related classes and methods.
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef PID_H
#define PID_H

namespace ls {
    /**
     * @brief The PID object, calculates PID outputs given an error.
     */
    class PID {
        public:
            // just to make c++ happy. Do not use this constructor for functionality
            explicit PID() {}
            /**
             * @brief Construct a new PID object
             * 
             * @param kP proportion constant
             * @param kI integral constant
             * @param kD derivative constant
             * @param windupRange desired windup range
             * @param signFlipReset if signed is fliped already.
             */
            explicit PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);
            /**
             * @brief calculate the new PID value given the error.
             * 
             * @param error the error from target
             * @return the value from PID
             */
            double update(const double error);
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
