#include <cstdlib>  // for random()
#include "Vector.h"

// return a random vector with x,y and z coordinate in the range [-1,1]

Vector3D Vector3D::init_random() {
    Vector3D res;

    res.x = 2.0 * ((double) random() / 0x7fffffff) - 1.0;
    res.y = 2.0 * ((double) random() / 0x7fffffff) - 1.0;
    res.z = 2.0 * ((double) random() / 0x7fffffff) - 1.0;

    return res;
}
