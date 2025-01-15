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
    float x=0.266, y=0, z=-0.29;
    float yaw, pitch, roll=-3.1415926;
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
    float a3=0.266, d4=0.28, d6=0.01;
    float offset5 = -0.349;

    Arm(Motor &m1, Motor &m2, Motor &m3, Motor &m4, Motor &m5, Motor &m6 )
        : m1(m1), m2(m2), m3(m3), m4(m4), m5(m5), m6(m6) {
    }
    void updateRefPose(Pose &ref_pose);
    void preventAngleJump(Joint &ref_joint, Joint &last_ref_joint);//防止atan2解算出现 180 跳变到-180
    void rc_to_theta();
    void get_joint();
    void handle();
    void ikine(Pose &ref_pose, Joint &ref_joint, Joint &last_ref_joint);
    void interpolate();
};


#ifdef __cplusplus
}
#endif
#endif //ARM_H
