/**
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

namespace ls {
    /**
     * @brief A structure that contains all parameters of the SmartPID class.
     * 
     * Order: correction_constant, windup, learning_constant, max_value, damp
     */
    struct smart_pid_parameters_t {
        double correction_constant;
        double windup;
        double learning_constant;
        double max_value;
        double damp;
    };

    /**
     * @brief The Smart PID class... an extension of the PID class that will adapt based on error. No need to provide constants.
     * 
     */
    class SmartPID {
        public:
            /**
             * @brief Construct a new SmartPID object
             * 
             * @param cc the correction constant. the rate at which it should ascend and decend to max/min power
             * @param w windup value for integral.
             * @param lc the learning constant. the rate at which it should learn
             * @param max the maximum value of the PID output.
             */
            explicit SmartPID(double cc, double w, double lc, double max, double damp);

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
            double update(const double e);

            /**
             * @brief Reset the speed, integral, and derivatives. Constants are not reset.
             * 
             */
            void reset();

        protected:
            double kp;
            double ki;
            double kd;
            double damp;
            
            double P;
            double I;
            double D;
            double prev_val;
            double prev_velocity = 0;
            double velocity = 0;
            
            double correction_constant;
            double windup;
            double learning_constant;
            double max_value;

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
             * @brief Gets the expected value for comparing to current state of PID value's to update constants to better adapt. (basicaly, train the PID constants)
             * 
             * @param e new error
             * @return double expected value
             */
            double get_expected(const double e);
    };
}


#endif // LS_SPID_H