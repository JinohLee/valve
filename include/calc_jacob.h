#ifndef CALC_JACOB_H
#define CALC_JACOB_H

#define WANT_STREAM                  // include.h will get stream fns
#define WANT_MATH                    // include.h will get math fns
                                     // newmatap.h will get include.h
#include "newmatap.h"                // need matrix applications
#include "newmatio.h"                // need matrix output routines


void calcJacob(const NEWMAT::Matrix &SC, NEWMAT::Matrix &jacobRelative);


#endif // CALC_JACOB_H
