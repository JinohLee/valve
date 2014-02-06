//////////////////////////////////////////////////
//  basic Computation for Orientation Control
//                          28-Jan-2014
//                          jinoh.lee@iit.it
//////////////////////////////////////////////////

#include <orientation_ctrl.h>
#include <mySkew.h>

//Unit Quaternion from a rotation matrix
void RotQuaternion(const mat &Rot, vec &Quat)
{
    Quat.zeros();
    double tmp1=0.0, tmp2=0.0, tmp3=0.0;
    tmp1 = sign(Rot(2,1)-Rot(1,2));
    tmp2 = sign(Rot(0,2)-Rot(2,0));
    tmp3 = sign(Rot(1,0)-Rot(0,1));

    if ( fabs(tmp1) < 0.0000001 ) tmp1 = 0.0;
    if ( fabs(tmp2) < 0.0000001 ) tmp2 = 0.0;
    if ( fabs(tmp2) < 0.0000001 ) tmp2 = 0.0;

    Quat<<1.0/2.0*sqrt(Rot(0,0)+Rot(1,1)+Rot(2,2)+1.0)
            <<1.0/2.0*(tmp1)*sqrt( Rot(0,0)-Rot(1,1)-Rot(2,2)+1.0 )
            <<1.0/2.0*(tmp2)*sqrt( Rot(1,1)-Rot(2,2)-Rot(0,0)+1.0 )
            <<1.0/2.0*(tmp3)*sqrt( Rot(2,2)-Rot(0,0)-Rot(1,1)+1.0 );

/*    Quat<<1.0/2.0*sqrt(Rot(0,0)+Rot(1,1)+Rot(2,2)+1.0)
        <<1.0/2.0*( sign(Rot(2,1)-Rot(1,2))*sqrt( Rot(0,0)-Rot(1,1)-Rot(2,2)+1.0) )
        <<1.0/2.0*( sign(Rot(0,2)-Rot(2,0))*sqrt( Rot(1,1)-Rot(2,2)-Rot(0,0)+1.0) )
        <<1.0/2.0*( sign(Rot(1,0)-Rot(0,1))*sqrt( Rot(2,2)-Rot(0,0)-Rot(1,1)+1.0) );
        */


}

//Orientation error ['Operational Space Control...,' IJRR]
void OrientationError(const vec &Qd, vec &Qe, vec &OrErr)
{
    mat Skew_Ed;
    OrErr.zeros();
    calcSkew(Qd.subvec(1,3), Skew_Ed);

    OrErr =  Qe(0)*Qd.subvec(1,3) -Qd(0)*Qe.subvec(1,3) - Skew_Ed*Qe.subvec(1,3);
    //eo_r = Qd.s*Q_r.v' - Q_r.s*Qd.v' + skew(Qd.v)*Q_r.v';
}
