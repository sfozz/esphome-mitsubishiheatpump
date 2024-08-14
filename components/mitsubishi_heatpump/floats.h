#include <cmath>

#ifndef FLOATSDS_H
#define FLOATSDS_H

namespace devicestate {

    static bool same_float(const float a, const float b, const float epsilon) {
        return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * epsilon);
    }

}

#endif