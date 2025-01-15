//
// Created by h on 2024/12/16.
//

#include "arm.h"

#include <stm32f4xx_hal.h>
#include <tgmath.h>
#include <base/remote/remote.h>

#include "rc_to_theta.h"
#include "base/monitor/motor_monitor.h"

// #include <iostream>
//
// using namespace std;


//pid 待调
Motor m1(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(150, 0, 0, 1000, 16384), Motor::None, 0);
Motor m2(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(150, 0, 0, 1000, 16384), Motor::None, 0);
Motor m3(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(150, 0, 0, 1000, 16384), Motor::None, 0);
Motor m4(Motor::M3508, 19.2, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
        PID(20, 0, 0, 1000, 16384), Motor::None, 0);
Motor m5(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
        PID(10, 0, 0, 1000, 5000), Motor::None, 0);
Motor m6(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(1, 0, 0, 1000, 5000),
        PID(1, 0, 0, 1000, 5000), Motor::None, 0);



Arm arm(m1, m2, m3, m4, m5, m6);


uint8_t ikineCnt = 0;
float ez6[3];
float o6[3];
float o5[3];
float z6[3];
float T36_23;

void Arm::ikine(Pose &ref_pose, Joint &ref_joint, Joint &last_ref_joint) {

    //解算o5
    ez6[0] = sin(ref_pose.roll)*sin(ref_pose.yaw) + cos(ref_pose.roll)*cos(ref_pose.yaw)*sin(ref_pose.pitch);
    ez6[1] = cos(ref_pose.roll)*sin(ref_pose.pitch)*sin(ref_pose.yaw) - cos(ref_pose.yaw)*sin(ref_pose.roll);
    ez6[2] = cos(ref_pose.pitch)*cos(ref_pose.roll);
    o6[0] = ref_pose.x;
    o6[1] = ref_pose.y;
    o6[2] = ref_pose.z;

    Vector3fScale(d6, ez6, z6);
    Vector3fSub(o6, z6, o5);

    float dist5 = o5[0]*o5[0] + o5[1]*o5[1] + o5[2]*o5[2];
    float max_dist5=0.81*(a3+d4)*(a3+d4);  //边界
    if (dist5>max_dist5) {
        float scale_factor = sqrt(max_dist5 / dist5);
        o5[0] *= scale_factor;
        o5[1] *= scale_factor;
        o5[2] *= scale_factor;
    }

    //由o5(x,y,z)解算q1,q2,q3
    //theta 1
    ref_joint.q[0] = atan2(o5[1],o5[0]);

    //theta 3
    float sin3 = (a3*a3 + d4*d4 - (o5[0]*cos(ref_joint.q[0]) + o5[1]*sin(ref_joint.q[0]))*(o5[0]*cos(ref_joint.q[0]) + o5[1]*sin(ref_joint.q[0])) - o5[2]*o5[2])/(2*a3*d4);
    float theta31 = atan2(sin3,+sqrt(1-sin3*sin3));
    float theta32 = atan2(sin3,-sqrt(1-sin3*sin3));

    //theta 2
    float theta21 = atan2(-(d4*cos(theta31)*(o5[0]*cos(ref_joint.q[0]) + o5[1]*sin(ref_joint.q[0])) + o5[2]*(a3 - d4*sin(theta31))),
    ((a3 - d4*sin(theta31))*(o5[0]*cos(ref_joint.q[0]) + o5[1]*sin(ref_joint.q[0])) - o5[2]*d4*cos(theta31)));
    float theta22 = atan2(-(d4*cos(theta32)*(o5[0]*cos(ref_joint.q[0]) + o5[1]*sin(ref_joint.q[0])) + o5[2]*(a3 - d4*sin(theta32))),
    ((a3 - d4*sin(theta32))*(o5[0]*cos(ref_joint.q[0]) + o5[1]*sin(ref_joint.q[0])) - o5[2]*d4*cos(theta32)));
    if (theta21 <= theta22) {
        ref_joint.q[1] = theta21;
        ref_joint.q[2] = theta31;
    }
    else {
        ref_joint.q[1] = theta22;
        ref_joint.q[2] = theta32;
    }

    //腕部奇异
    T36_23 = cos(ref_pose.pitch)*cos(ref_pose.roll)*sin(ref_joint.q[1])*sin(ref_joint.q[2]) - cos(ref_pose.pitch)*cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[2]) - cos(ref_joint.q[0])*cos(ref_joint.q[1])*sin(ref_pose.roll)*sin(ref_joint.q[2])*sin(ref_pose.yaw) - cos(ref_joint.q[0])*cos(ref_joint.q[2])*sin(ref_pose.roll)*sin(ref_joint.q[1])*sin(ref_pose.yaw) + cos(ref_joint.q[1])*cos(ref_pose.yaw)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[2]) + cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1]) - cos(ref_pose.roll)*cos(ref_joint.q[0])*cos(ref_joint.q[1])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[2]) - cos(ref_pose.roll)*cos(ref_joint.q[0])*cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[1]) - cos(ref_pose.roll)*cos(ref_joint.q[1])*sin(ref_pose.pitch)*sin(ref_joint.q[0])*sin(ref_joint.q[2])*sin(ref_pose.yaw) - cos(ref_pose.roll)*cos(ref_joint.q[2])*sin(ref_pose.pitch)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_pose.yaw);
    if (T36_23 < 0.9) {
         // theta 6
         ref_joint.q[5] = atan2(
             -(cos(ref_pose.pitch) * sin(ref_pose.roll) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) -
               cos(ref_pose.pitch) * cos(ref_joint.q[1]) * cos(ref_joint.q[2]) * sin(ref_pose.roll) +
               cos(ref_pose.roll) * cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * sin(ref_joint.q[2]) * sin(ref_pose.yaw) +
               cos(ref_pose.roll) * cos(ref_joint.q[0]) * cos(ref_joint.q[2]) * sin(ref_joint.q[1]) * sin(ref_pose.yaw) -
               cos(ref_pose.roll) * cos(ref_joint.q[1]) * cos(ref_pose.yaw) * sin(ref_joint.q[0]) * sin(ref_joint.q[2]) -
               cos(ref_pose.roll) * cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_joint.q[0]) * sin(ref_joint.q[1]) -
               cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_pose.roll) * sin(ref_joint.q[2]) -
               cos(ref_joint.q[0]) * cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_pose.roll) * sin(ref_joint.q[1]) -
               cos(ref_joint.q[1]) * sin(ref_pose.pitch) * sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_joint.q[2]) * sin(ref_pose.yaw) -
               cos(ref_joint.q[2]) * sin(ref_pose.pitch) * sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_joint.q[1]) * sin(ref_pose.yaw)),
             (cos(ref_joint.q[1]) * cos(ref_joint.q[2]) * sin(ref_pose.pitch) - sin(ref_pose.pitch) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) -
              cos(ref_pose.pitch) * cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * cos(ref_pose.yaw) * sin(ref_joint.q[2]) -
              cos(ref_pose.pitch) * cos(ref_joint.q[0]) * cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_joint.q[1]) -
              cos(ref_pose.pitch) * cos(ref_joint.q[1]) * sin(ref_pose.yaw) * sin(ref_joint.q[0]) * sin(ref_joint.q[2]) -
              cos(ref_pose.pitch) * cos(ref_joint.q[2]) * sin(ref_pose.yaw) * sin(ref_joint.q[0]) * sin(ref_joint.q[1])));

         // theta 4
         ref_joint.q[3] = atan2(
             (cos(ref_pose.roll) * cos(ref_joint.q[0]) * sin(ref_pose.pitch) * sin(ref_pose.yaw) -
              cos(ref_joint.q[0]) * cos(ref_pose.yaw) * sin(ref_pose.roll) -
              sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_pose.yaw) -
              cos(ref_pose.roll) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_joint.q[0])),
             -(cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * cos(ref_joint.q[2]) * sin(ref_pose.roll) * sin(ref_pose.yaw) -
              cos(ref_pose.pitch) * cos(ref_pose.roll) * cos(ref_joint.q[2]) * sin(ref_joint.q[1]) -
              cos(ref_pose.pitch) * cos(ref_pose.roll) * cos(ref_joint.q[1]) * sin(ref_joint.q[2]) -
              cos(ref_joint.q[1]) * cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_pose.roll) * sin(ref_joint.q[0]) -
              cos(ref_joint.q[0]) * sin(ref_pose.roll) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) * sin(ref_pose.yaw) +
              cos(ref_pose.yaw) * sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) +
              cos(ref_pose.roll) * cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_pose.pitch) -
              cos(ref_pose.roll) * cos(ref_joint.q[0]) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) +
              cos(ref_pose.roll) * cos(ref_joint.q[1]) * cos(ref_joint.q[2]) * sin(ref_pose.pitch) * sin(ref_joint.q[0]) * sin(ref_pose.yaw) -
              cos(ref_pose.roll) * sin(ref_pose.pitch) * sin(ref_joint.q[0]) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) * sin(ref_pose.yaw)));

         // theta 5
         ref_joint.q[4] = atan2(
             (cos(ref_pose.roll) * cos(ref_joint.q[0]) * sin(ref_pose.pitch) * sin(ref_pose.yaw) -
              cos(ref_joint.q[0]) * cos(ref_pose.yaw) * sin(ref_pose.roll) -
              sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_pose.yaw) -
              cos(ref_pose.roll) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_joint.q[0])) / sin(ref_joint.q[3]),
             (cos(ref_pose.pitch) * cos(ref_pose.roll) * sin(ref_joint.q[1]) * sin(ref_joint.q[2]) -
              cos(ref_pose.pitch) * cos(ref_pose.roll) * cos(ref_joint.q[1]) * cos(ref_joint.q[2]) -
              cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * sin(ref_pose.roll) * sin(ref_joint.q[2]) * sin(ref_pose.yaw) -
              cos(ref_joint.q[0]) * cos(ref_joint.q[2]) * sin(ref_pose.roll) * sin(ref_joint.q[1]) * sin(ref_pose.yaw) +
              cos(ref_joint.q[1]) * cos(ref_pose.yaw) * sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_joint.q[2]) +
              cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_pose.roll) * sin(ref_joint.q[0]) * sin(ref_joint.q[1]) -
              cos(ref_pose.roll) * cos(ref_joint.q[0]) * cos(ref_joint.q[1]) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_joint.q[2]) -
              cos(ref_pose.roll) * cos(ref_joint.q[0]) * cos(ref_joint.q[2]) * cos(ref_pose.yaw) * sin(ref_pose.pitch) * sin(ref_joint.q[1]) -
              cos(ref_pose.roll) * cos(ref_joint.q[1]) * sin(ref_pose.pitch) * sin(ref_joint.q[0]) * sin(ref_joint.q[2]) * sin(ref_pose.yaw) -
              cos(ref_pose.roll) * cos(ref_joint.q[2]) * sin(ref_pose.pitch) * sin(ref_joint.q[0]) * sin(ref_joint.q[1]) * sin(ref_pose.yaw)));
     }
    else {
        ref_joint.q[3] = last_ref_joint.q[3]/57.3;
        ref_joint.q[4] = last_ref_joint.q[4]/57.3;
        ref_joint.q[5] = last_ref_joint.q[5]/57.3;
    }
    for (int i = 0; i < 6; i++) {
        ref_joint.q[i] *= 57.3;
    }
}

void Arm::preventAngleJump(Joint &ref_joint, Joint &last_ref_joint) {
    int i;
    for(i = 0;i < 6;i++) {
        if ((ref_joint.q[i] - last_ref_joint.q[i]) <= -200) {
            ref_joint.q[i] += 360;
        }
        else if ((ref_joint.q[i] - last_ref_joint.q[i]) >= 200) {
            ref_joint.q[i] -= 360;
        }
    }
    last_ref_joint = ref_joint;
}


float ANGLE_INCREMENT=0.0005;
float PITCH_YAW_INCREMENT=0.001;
extern RC rc;
void Arm::updateRefPose(Pose &rc_pose) {
    if (rc.switch_.l == 0) {
        rc_pose.x += rc.channel_.r_col * ANGLE_INCREMENT; //单位m
        rc_pose.y -= rc.channel_.r_row * ANGLE_INCREMENT;
        rc_pose.z += rc.channel_.l_col * ANGLE_INCREMENT;
    } else if (rc.switch_.l == 1) {
        rc_pose.pitch += rc.channel_.r_col * PITCH_YAW_INCREMENT;//单位rad
        rc_pose.yaw += rc.channel_.r_row * PITCH_YAW_INCREMENT;
        rc_pose.roll += rc.channel_.l_row * PITCH_YAW_INCREMENT;
    }
}

Joint ref_joint;
Joint last_ref_joint;

Pose ref_pose;
void Arm::handle() {

    updateRefPose(ref_pose);

    ikine(ref_pose, ref_joint, last_ref_joint);
    preventAngleJump(ref_joint, last_ref_joint);

    m1.setAngle(ref_joint.q[0],0);
    m2.setAngle(-ref_joint.q[1],0);
    m3.setAngle(ref_joint.q[2],0);
    m4.setAngle(ref_joint.q[3],0);
    m5.setAngle(-ref_joint.q[4],0);
    m6.setAngle(ref_joint.q[5],0);

}

