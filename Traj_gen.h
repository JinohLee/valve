#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include <armadillo>
using namespace arma;

void line_traj(const vec &Xinit, vec &Xf, double Tf, double t, vec &Xd, vec &dXd);
double circle_traj(const vec &Xinit, double center_angle, double Tf, double t, double Radius, vec &Xd, vec &dXd);

//void line_traj_RH(const vec &Xinit, vec &X, double Tf, double t);
//void line_traj_LH(const vec &Xinit, vec &X, double Tf, double t);

#endif // TRAJ_GEN_H
