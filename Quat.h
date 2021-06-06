#ifndef QUAT_H
#define QUAT_H

#include <arduino.h>

#ifndef QUAT_PRECICION
#define QUAT_PRECISION 0
#endif

#if QUAT_PRECISION == 0
#define PREC float
#else
#define PREC double
#endif

class Quat{
public:
    PREC w;
    PREC i;
    PREC j;
    PREC k;
    
    Quat(PREC w, PREC i, PREC j, PREC k);
    Quat();
    
    void norm(PREC tolerance = 0.0001);
    void normTo(PREC length, PREC tolerance = 0.0001);
    void fromAngleVec(PREC angle, PREC heading, PREC elevation);
    void fromGyro(PREC x, PREC y, PREC z); // TODO: make this take arrays
    
    PREC getLength();
    
    Quat inverse();
    Quat copy();
    static Quat mult(Quat q1, Quat q2);
    static Quat rot(Quat point, Quat rotator);
    static Quat add(Quat q1, Quat q2);
    static Quat sub(Quat q1, Quat q2);
};

#endif