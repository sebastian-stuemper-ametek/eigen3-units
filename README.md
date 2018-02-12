# eigen3-units

[![Build Status](https://travis-ci.org/iastate-robotics/eigen3-units.svg?branch=master)](https://travis-ci.org/iastate-robotics/eigen3-units)
[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](../master/LICENSE)

A compile time units conversion and dimension checking header file for Eigen3

```
Time:
6 s

Force 1:
  1 N
  3 N
  5 N

Force 1 Transpose:
  1 N   3 N   5 N

Displacement 1 Transpose:
  1 m   2 m   2 m

Force 1 Columnwise Square:
       1 m^2 kg^2 s^-4
       9 m^2 kg^2 s^-4
      25 m^2 kg^2 s^-4

Force 1 Columnwise Square Root:
        1 m^(1/2) kg^(1/2) s^-1
  1.73205 m^(1/2) kg^(1/2) s^-1
  2.23607 m^(1/2) kg^(1/2) s^-1

f1.dot(r1):
17 J

f1.cross(r1):
  -4 J
   3 J
  -1 J

r1.dot(f1):
17 J

r1.cross(f1):
   4 J
  -3 J
   1 J

r1 + r2 =
  3 m
  6 m
  6 m

r1 - r2 =
  -1 m
  -2 m
  -2 m

r1 * 5.0 =
   5 m
  10 m
  10 m

r1.sum() =
5 m

Acceleration 1 (a1) =
   4 m s^-2
   4 m s^-2
   5 m s^-2

v2 = a1 * t1 + v1 =
   25 m s^-1
   26 m s^-1
   33 m s^-1

v2 * t1 + r1 =
  151 m
  158 m
  200 m

AngularVelocity (theta_dot):
   3 Hz
   2 Hz
   1 Hz

AngularAcceleration (omega):
     3 s^-2
     2 s^-2
     1 s^-2

theta =
    3.13 dimensionless
    1.13 dimensionless
    0.23 dimensionless

omega * t * t + theta_dot * t + theta = :
  129.13 dimensionless
   85.13 dimensionless
   42.23 dimensionless

r1 * 5.0
   5 m
  10 m
  10 m

5.0 * r1
   5 m
  10 m
  10 m

r1 / 5.0
  0.2 m
  0.4 m
  0.4 m

util::squared_norm(r1) =
9 m^2

util::norm(r1) =
3 m

util::normalized(r1) =
  0.333333 m
  0.666667 m
  0.666667 m⏎
```