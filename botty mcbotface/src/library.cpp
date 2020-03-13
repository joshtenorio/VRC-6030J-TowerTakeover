/**
 *  @file src/library.cpp
 */
#include "main.h"

int findSign(double num){
    if(num > 0) return 1;
    else if(num < 0) return -1;
    else return 0;
}

double vectorDotProduct(Vector u, Vector v){
    return u.x * v.x + u.y * v.y;
}
Vector vectorNormalize(Vector u){
    double vectorLength = sqrt(u.x * u.x + u.y * u.y);

    return Vector{u.x/vectorLength, u.y/vectorLength};
}

double vectorMagnitude(Vector u){
    return sqrt(u.x * u.x + u.y * u.y);
}

double distanceFormula(Point p, Point q){
    double xDiff = q.x - p.x;
    double yDiff = q.y - p.y;
    return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

double squareNum(double num){
    return num * num;
}

double min(double a, double b){
    if(a > b) return b;
    else if(b > a) return a;
    else return a;
}

double constrain(double input, double min, double max){
    if(min > input) return min;
    else if(input > max) return max;
    else return input;
}

int closestInt(double num){
    return (int) (num + 0.5);
}

double closestEquivAngle(double refAngle, double targetAngle){
    return closestInt((refAngle - targetAngle) / (2 * PI)) * (2 * PI) + targetAngle;
}

double constrainAngle(double angle){
    double x = fmod(angle, 2.0 * PI);
    if (x < 0){
        x += 2.0 * PI;
    }
    return x;
}


