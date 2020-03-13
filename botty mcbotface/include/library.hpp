/**
 *  @file include/library.hpp
 */
#pragma once
#include "robot/odometry.hpp"

typedef struct {
    double x;
    double y;
} Vector;

int findSign(double num);
double vectorDotProduct(Vector u, Vector v);
Vector vectorNormalize(Vector u);
double vectorMagnitude(Vector u);

double distanceFormula(Point p, Point q);
double squareNum(double num);
int closestInt(double num);
double min(double a, double b);
double constrain(double input, double min, double max);

double closestEquivAngle(double refAngle, double targetAngle);
double constrainAngle(double angle);

