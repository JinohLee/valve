//////////////////////////////////////////////////
//  Trajectory Generation
//                          22-Jan-2014
//                          jinoh.lee@iit.it
//////////////////////////////////////////////////
//(initial position, displacement, time inverval, time)

#include <Traj_gen.h>

void line_traj(const vec &Xinit, vec &Xf, double Tf, double t, vec &Xd, vec &dXd)
{
    //Displacement
    //vec Xf(6); Xf.zeros();
    //vec Xd(6), dXd(6);  //Xd.zeros(); dXd.zeros();
    //Xf = X; //displacement = target position - initial position

    if(t>=0.0 && t<=Tf) {
        Xd = Xinit+20.0*Xf/(2.0*Tf*Tf*Tf)*t*t*t+(-30.0*Xf)/(2.0*Tf*Tf*Tf*Tf)*t*t*t*t+(12.0*Xf)/(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t*t*t;
        dXd = 60.0*Xf/(2.0*Tf*Tf*Tf)*t*t + 4.0*(-30.0*Xf)/(2.0*Tf*Tf*Tf*Tf)*t*t*t+5.0*(12.0*Xf)/(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t*t;
        //ddXd=120.0*Xf/(2.0*Tf*Tf*Tf)*t + 12.0*(-30.0*Xf)/(2.0*Tf*Tf*Tf*Tf)*t*t+20.0*(12.0*Xf)/(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t;
    }
    else if(t>Tf){
        Xd = Xinit+Xf;
        dXd.zeros();
        //ddXd = 0.;
    }
}

double circle_traj(const vec &Xinit, double center_angle, double Tf, double t, double Radius, vec &Xd, vec &dXd)
{
    double Xf1;
    vec Xd_v(6), dXd_v(6);
    Xd_v.zeros();   dXd_v.zeros();
    double CircleAngle=0.0, DCircleAngle=0.0, DDCircleAngle=0.0;
    Xf1 = center_angle;

    if(t>=0.0 && t<=Tf){
        CircleAngle=20.0*Xf1/(2.0*Tf*Tf*Tf)*t*t*t+(-30.0*Xf1)/(2.0*Tf*Tf*Tf*Tf)*t*t*t*t+(12.0*Xf1)/(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t*t*t;
        DCircleAngle=60.0*Xf1/(2.0*Tf*Tf*Tf)*t*t + 4.0*(-30.0*Xf1)/(2.0*Tf*Tf*Tf*Tf)*t*t*t+5.0*(12.0*Xf1)/(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t*t;
        //DDCircleAngle=120.0*Xf1/(2.0*Tf*Tf*Tf)*t + 12.0*(-30.0*Xf1)/(2.0*Tf*Tf*Tf*Tf)*t*t+20.0*(12.0*Xf1)/(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t;

        //Xv axis
        Xd_v(0) = 0.0; Xd_v(0) = 0.0;

        //Yv axis
        Xd_v(1)	 = Radius*(1.0 - cos(CircleAngle) );
        dXd_v(1) = Radius*sin(CircleAngle)*DCircleAngle;
        //ddXrd(1)= Radius*cos(CircleAngle)*DCircleAngle*DCircleAngle + Radius*sin(CircleAngle)*DDCircleAngle;

        //Zv axis
        Xd_v(2)  = Radius*(sin(CircleAngle));
        dXd_v(2) = Radius*cos(CircleAngle)*DCircleAngle;
        //ddXd(2)=-Radius*sin(CircleAngle)*DCircleAngle*DCircleAngle + Radius*cos(CircleAngle)*DDCircleAngle;


        //Transformation Xd_v to Xd
        //position
        Xd = Xinit + Xd_v;  //Xd = Xinit + Rot(o,v)*Xd_v;

        //orientation
        //rotation matrix --> euler angle
        //Xd(3) = Xinit(3); 	    Xd(4) = Xinit(4);     Xd(5) = Xinit(5);
        //dXd(3)=0.0; dXd(4)=0.0; dXd(5)=0.0;
        //ddXd(3)=0.0; ddXd(4)=0.0; ddXd(5)=0.0;
    }
    else{
        Xd  = Xinit;
        Xd(1)  = Xinit(1) + Radius*(1.0 - cos(Xf1) );
        Xd(2)  = Xinit(2) + Radius*(sin(Xf1));

        dXd.zeros();

        CircleAngle = Xf1;
        //ddXd.zeros();
    }

    return CircleAngle;

}
