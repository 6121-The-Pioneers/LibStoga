/** ✅
 * @file spid.h
 * @author Rishit Varshney
 * @brief Contains all code for the SPID (Smart PID) Algorithm
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef LS_SPID_H
#define LS_SPID_H

#include "pid.h"
#include <bits/stdc++.h>

namespace ls {
    /**
     * @brief A structure that contains all parameters of the SmartPID class.
     * 
     * Order: correction_constant, windup, learning_constant, max_value, damp
     */
    struct smart_pid_parameters_t {
        std::function<double()> actual_output;
        double correction_constant = 1;
        double windup = 1;
        double learning_constant = 0.001;
        double max_value = 127;
    };

    /**
     * @brief The Smart PID class... an extension of the PID class that uses the Smart PID Algorithm.
     * More about SPID can be looked up in the LibStoga documentation.
     * 
     */
    class SmartPID : public ls::AbstractPID {
        public:
            /**
             * *Attention* - This algorithm creates really fast changes in output which can be dangerous to the robot if correct parameters are not provided. change the settings with seperate warnings only if you know what you are doing.
             * More about SPID can be looked up in the LibStoga documentation.
             * 
             * @brief Construct a new SmartPID object
             * 
             * @param actual the function that provides the actual control output of the system. FUNCTION MUST OUTPUT A VALID DOUBLE IN THE RANGE OF [-max, max]
             * @param cc the correction constant. determins the speed of the overall motion.
             * @param w windup value for integral. will also determine how much the integral will be dominant in the motion. USE ONLY IF YOU KNOW WHAT YOU ARE DOING.
             * @param lc the learning constant. the rate at which it should learn. CHANGE ONLY IF YOU KNOW WHAT YOU ARE DOING.
             * @param max the maximum value of the PID output.
             * 
             */
            explicit SmartPID(std::function<double()> actual, double cc = 1, double w = 10, double lc = 0.001, double max = 127);

            /**
             * @brief Construct a new SmartPID object
             * 
             * @param parameters parameter that contains all values given by struct.
             */
            explicit SmartPID(smart_pid_parameters_t& parameters);

            /**
             * @brief Calculate the new PID value given the error.
             * 
             * @param error the error from target
             * @return the value from PID
             */
            double update(const double e) override;

            /**
             * @brief Reset the speed, integral, and derivatives. Constants are not reset.
             * 
             */
            void reset() override;

        private:
            double kP;
            double kI;
            double kD;
            
            double P;
            double I;
            double D;
            double prev_val;
            
            double correction_constant; // affects shape of fitting curve
            double windup; // integral max/min value
            double learning_constant; // affects learning rate
            double max_value; // maximum value control_output can ever be

            std::function<double()> actual_output;
            double control_output;

            /**
             * @brief Updates the proportion, integral, and derivative components of normal PID.
             * 
             * @param e new error
             */
            void update_components(const double e);
            
            /**
             * @brief Updates the constants based on calculated components for Smart PID.
             * 
             * @param e new error
             */
            void update_constants(const double e);

            /**
             * @brief Update mu (latency constant)
             */
            double update_mu();
            
            /**
             * @brief Gets the expected value for comparing to current state of PID value's to update constants to better adapt. (basicaly, train the PID constants)
             * 
             * @param e new error
             * @return double expected value
             */
            double get_expected(const double e) const;

            /**
             * @brief Special sigmoid function used to reduce learning constants when they get closer 0
             * 
             * @param x value to operate on
             * @return double sigmoid-ed value
            */
            double learn_sigmoid(double x) const;
    };
}


#endif // LS_SPID_H