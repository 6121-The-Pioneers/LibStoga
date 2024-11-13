#ifndef ERROR_LS_H
#define ERROR_LS_H

#include "api.h"

#define THROW_ERROR(error) __throw_error(error, __LINE__, __FILE__)

namespace ls {
    inline void __throw_error(char* error, unsigned long line, char* file) {
        pros::lcd::clear_line(0);
        pros::lcd::print(0, "at line %lu, file %s", line, file);
        pros::lcd::clear_line(1);
        pros::lcd::print(1, error);
        while (true) pros::delay(1000);
    }
};



#endif // ERROR_LS_H