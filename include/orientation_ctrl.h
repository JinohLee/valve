#ifndef ORIENTATION_CTRL_H
#define ORIENTATION_CTRL_H

#include <armadillo>
using namespace arma;

#include <boost/math/special_functions/sign.hpp>
using namespace boost::math;

void RotQuaternion(const mat &Rot, vec &Q);
void OrientationError(const vec &Qd, vec &Qe, vec &OrErr);

#endif // ORIENTATION_CTRL_H
