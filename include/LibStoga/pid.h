/** ✅
 * @file pid.h
 * @author Rishit Varshney
 * @brief Contains all PID related classes and methods.
 * @version 0.1
 * @date 2024-11-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef PID_H
#define PID_H

#include "error.h"
#include "gains.h"

namespace ls {

    /*
    * @brief abstract class for PID
    */
    class AbstractPID {
    public:
        /*
        * @brief abstract
        */
        virtual double update(const double error) = 0;
        
        /*
        * @brief abstract
        */
        virtual void reset() = 0;

        /**
         * @brief Set new PID gains.
         * 
         * @param gains The new gains to use.
         */
        virtual void setGains(const PIDGains& gains) = 0;

        /**
         * @brief Get current PID gains.
         * 
         * @return The current PID gains.
         */
        virtual PIDGains getGains() const = 0;
    };


    /**
     * @brief The PID object, calculates PID outputs given an error.
     */
    class PID : public AbstractPID {
        public:
            // just to make c++ happy. Do not use this constructor for functionality
            explicit PID() { THROW_ERROR("PID Contructor requires paramters"); }
            /**
             * @brief Construct a new PID object
             * 
             * @param kP proportion constant
             * @param kI integral constant
             * @param kD derivative constant
             * @param windupRange desired windup range
             * @param signFlipReset if signed is fliped already.
             */
            explicit PID(float kP, float kI, float kD, float windupRange, bool signFlipReset, float integralLimit = 1e9);
            
            /**
         * @brief Construct a new PID object from a gains struct
         * 
         * @param gains The PID gains
         */
        explicit PID(const PIDGains& gains);

            /**
             * @brief calculate the new PID value given the error.
             * 
             * @param error the error from target
             * @return the value from PID
             */
            double update(const double error) override;
            
            /**
             * @brief Resets the integral and other variables to reset PID state to original. 
             */
            void reset() override;

            /**
             * @brief Sets new gains for the PID controller.
             * 
             * @param gains The new PID gains.
             */
            void setGains(const PIDGains& gains) override;

            /**
             * @brief Gets the current PID gains.
             * 
             * @return The current PID gains.
             */
            PIDGains getGains() const override;

        protected:
            float kP;
            float kI;
            float kD;
            float windupRange;
            bool signFlipReset;
            float integral;
            float prevError;
            float integralLimit;
            unsigned long lastTimeMs{0};
        };
} 

#endif
