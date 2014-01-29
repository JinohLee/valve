//////////////////////////////////////////////////
//  Skew Symmetric Matrix
//
//  written by Arash Ajoudani
//  bug fixed (sign)

#include "mySkew.h"

void calcSkew(const vec &befScew, mat &aftScew)
{
    aftScew.zeros();

    aftScew<<0.0<<-befScew(2)<<befScew(1)<<endr
           <<befScew(2)<<0.0<<-befScew(0)<<endr
           <<-befScew(1)<<befScew(0)<<0.0<<endr;

}

