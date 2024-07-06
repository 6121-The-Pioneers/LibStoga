#ifndef GEOMETRY_LS_H
#define GEOMETRY_LS_H


namespace ls {
    /**
     * @class Angle
     * @brief A class to represent and manipulate angles.
     */
    class Angle {
        private:
            double angle;
        public:
            /**
             * @brief Default constructor initializes the angle to 0 degrees.
             */
            Angle(): angle(0) {}

            /**
             * @brief Parameterized constructor initializes the angle to the given value.
             * @param a The initial value of the angle in degrees.
             */
            Angle(double a): angle(a) {}

            /**
             * @brief Sets the angle to the specified value.
             * @param val The new value of the angle in degrees.
             */
            void setAngle(double val);

            /**
             * @brief Returns the current value of the angle.
             * @return The current angle in degrees.
             */
            double getAngle() const;

            /**
             * @brief Converts the angle from degrees to radians.
             * @return The angle in radians.
             */
            double convertToRadians() const;

            /**
             * @brief Normalizes the angle to be within the range [0, 360) degrees.
             * @return The normalized angle in degrees.
             */
            double normalize() const;

            /**
             * @brief returns the MINIMUM difference between this angle and the given angle.
             * this difference will have a range of [-180, 180] and is most useful for finding
             * the error for turn PID.
             * 
             * @param angle the angle to compare
             * @return the angle difference.
             */
            Angle minimumAngleDifference(Angle& angle) const; 

            /**
             * @brief Another way to use the setAngle() method.
             * 
             * @param x the new value of this angle.
             */
            void operator=(double x);
            
            /**
             * @brief increemnts this angle by 'other' angle numricaly.
             * 
             * @param other the other angle to increment by.
             */
            void operator+=(Angle other);
    };

    /**
     * @brief Converts a given angle in degrees to radians.
     * @param degrees The angle in degrees to be converted.
     * @return The angle in radians.
     */
    double degreesToRadians(double degrees);

    /**
     * @brief Converts the current angle from radians to degrees.
     * @param radians The angle in radians to be converted.
     * @return The angle in degrees.
     */
    double radiansToDegrees(double radians);

}


#endif // GEOMETRY_LS_H