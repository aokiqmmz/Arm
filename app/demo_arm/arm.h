//
// Created by h on 2024/12/16.
//

#ifndef ARM_H
#define ARM_H
#include <algorithm/math/matrix.h>
#include <base/motor/motor.h>

#ifdef __cplusplus
extern "C" {
#endif


class Pose {
public:
    float x=0.01, y=0, z=0.5;
    float yaw, pitch, roll;
};


class Joint {
public:
    float q[6];

    Joint() : q() {}
};

class Arm {
public:
    Motor &m1, &m2, &m3, &m4, &m5, &m6;
    Joint arm_joint;
    //Pose ref_pose;
    float l1=5, l2=40, l3=30;
    float offset3 = 3.1415926/2;
    float offset4 = -3.1415926/2;
    float offset5 = 71.8945312/180 * 3.1415926;

    Arm(Motor &m1, Motor &m2, Motor &m3, Motor &m4, Motor &m5, Motor &m6 )
        : m1(m1), m2(m2), m3(m3), m4(m4), m5(m5), m6(m6) {
    }
    void updateRefPose(Pose &ref_pose);
    void rc_to_theta();
    void get_joint();
    void handle();
    void ikine(Pose &ref_pose, Pose &rc_pose, Joint &ref_joint);
    void interpolate();
};

class Trajectory {
public:
    Joint start_joint;
    Joint end_joint;
    Joint interpolate_joint;
    float sigma;
    float ticks = 0.01;
    float start_ticks;

    Trajectory(Joint start_joint, Joint end_joint)
        : start_joint(start_joint), end_joint(end_joint) {}

    void handle();
};


#ifdef __cplusplus
}
#endif
#endif //ARM_H
