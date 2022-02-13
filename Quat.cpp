#include "arduino.h"
#include "quat.h"

PREC w;
PREC i;
PREC j;
PREC k;

Quat::Quat(PREC w, PREC i, PREC j, PREC k){
    this->w = w;
    this->i = i;
    this->j = j;
    this->k = k;
}

Quat::Quat(){
    w=0;i=0;j=0;k=0;
}

void Quat::normTo(PREC length, PREC tolerance){
    PREC bigness = sqrt(w*w + i*i + j*j + k*k);
    if(abs(length - bigness) >= tolerance){
        w *= length/bigness;
        i *= length/bigness;
        j *= length/bigness;
        k *= length/bigness;
    }
}

void Quat::norm(PREC tolerance){
    Quat::normTo(1, tolerance);
}

void Quat::fromAngleVec(PREC angle, PREC heading, PREC elevation){
    angle *= 0.5;
    this->w = cos(angle);
    this->i = cos(heading) * cos(elevation) * sin(angle);
    this->j = sin(elevation) * sin(angle);
    this->k = sin(heading) * cos(elevation) * sin(angle);
}

void Quat::fromGyro(PREC x, PREC y, PREC z){
    //dont ask me how this works i have no clue
    //i found it on a stanford pdf, so it must be good
    //i dont even know what the gyro data means
    //oh also it wants radians
    PREC angle = sqrt(x*x + y*y + z*z); // magnitude of rotation vector or something
    if((float)angle != 0.f){ // can't divide by zero!
        this->w = cos(angle/2);
        this->i = x/angle * sin(angle/2);
        this->j = y/angle * sin(angle/2);
        this->k = z/angle * sin(angle/2);
    }else{
        this->w = 1;
        this->i = 0;
        this->j = 0;
        this->k = 0;
  }
}

PREC getLength(){
    return sqrt(w*w + i*i + j*j + k*k);
}

Quat Quat::inverse(){
    return Quat(-w, i, j ,k);
}

Quat Quat::copy(){
    return Quat(w, i ,j ,k);
}

Quat Quat::mult(Quat q1, Quat q2){
    return Quat(
    q1.w*q2.w - q1.i*q2.i - q1.j*q2.j - q1.k*q2.k,
    q1.w*q2.i + q1.i*q2.w + q1.j*q2.k - q1.k*q2.j,
    q1.w*q2.j - q1.i*q2.k + q1.j*q2.w + q1.k*q2.i,
    q1.w*q2.k + q1.i*q2.j - q1.j*q2.i + q1.k*q2.w);
}

Quat Quat::add(Quat q1, Quat q2){
    return Quat(
    q1.w + q2.w,
    q1.i + q2.i,
    q1.j + q2.j,
    q1.k + q2.k);
}

Quat Quat::sub(Quat q1, Quat q2){
    return Quat(
    q1.w - q2.w,
    q1.i - q2.i,
    q1.j - q2.j,
    q1.k - q2.k);
}

Quat Quat::rot(Quat point, Quat rotation){
    Quat ret = mult(rotation, point);
    ret = mult(ret, rotation);
    return ret;
}