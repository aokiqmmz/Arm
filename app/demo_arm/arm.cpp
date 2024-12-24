//
// Created by h on 2024/12/16.
//

#include "arm.h"

#include <stm32f4xx_hal.h>
#include <tgmath.h>
#include <base/remote/remote.h>

#include "rc_to_theta.h"
#include "base/monitor/motor_monitor.h"


//pid 待调
Motor m1(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(200, 0, 0, 1000, 16384), Motor::None, 0);
Motor m2(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(200, 0, 0, 1000, 16384), Motor::None, 0);
Motor m3(Motor::M3508, 100, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(200, 0, 0, 1000, 16384), Motor::None, 0);
Motor m4(Motor::M3508, 19.2, Motor::POSITION_SPEED,
        PID(20, 0, 0, 1000, 5000),
        PID(20, 0, 0, 1000, 16384), Motor::None, 0);
Motor m5(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
        PID(10, 0, 0, 1000, 5000), Motor::None, 0);
Motor m6(Motor::M3508, 1, Motor::POSITION_SPEED,
        PID(10, 0, 0, 1000, 5000),
        PID(10, 0, 0, 1000, 5000), Motor::None, 0);




Arm arm(m1, m2, m3, m4, m5, m6);



void Trajectory::handle() {
    start_ticks = HAL_GetTick();
    sigma = (float)(HAL_GetTick() - start_ticks) / (float)ticks;
    for (int i = 0; i < 6; ++i) {
        interpolate_joint.q[i] = sigma * end_joint.q[i] + (1 - sigma) * start_joint.q[i];
    }
}

void Arm::ikine(Pose &ref_pose, Joint &ref_joint) {
  float dist=ref_pose.x*ref_pose.x+ref_pose.y*ref_pose.y+ref_pose.z*ref_pose.z;//距离过大时，指向要达到的那个点
  float max_dist=0.99*(l1+l2)*(l1+l2);
  float control_x=ref_pose.x,control_y=ref_pose.y,control_z=ref_pose.z;
  if(dist>max_dist) {
    control_x=ref_pose.x*max_dist/dist;
    control_y=ref_pose.y*max_dist/dist;
    control_z=ref_pose.z*max_dist/dist;
  }

  if(ref_pose.x<0.01&&ref_pose.x>-0.01) control_x=0.01;
  if(ref_pose.y<0.01&&ref_pose.y>-0.01) control_y=0.01;
  if(ref_pose.z<0.01&&ref_pose.z>-0.01) control_z=0.01;

    //theta 1
    //theta 1 has 2 solutions, on choisit la solution la plus proche(we choose the closer solution)
    float theta11 = atan2(control_y,control_x) - atan2(l1/sqrt(control_x*control_x + control_y*control_y), +sqrt(1-pow((l1/sqrt(control_x*control_x + control_y*control_y)),2)));
    float theta12 = atan2(control_y,control_x) - atan2(l1/sqrt(control_x*control_x + control_y*control_y), -sqrt(1-pow((l1/sqrt(control_x*control_x + control_y*control_y)),2)));
    if (abs(theta11-ref_joint.q[0]) < abs(theta12-ref_joint.q[0])) {
        ref_joint.q[0] = theta11;
    }
    else {
        ref_joint.q[0] = theta12;
    }

    //theta 3
    //theta 3 has 2 solutions, on choisit la solution la plus proche(we choose the closer solution)
    float sin3 = -(pow((control_x*cos(ref_joint.q[0]) + control_y*sin(ref_joint.q[0])),2) - l2*l2 - l3*l3 + control_z*control_z)/(2*l2*l3);
    float theta31 = atan2(sin3, +sqrt(1-sin3*sin3));
    float theta32 = atan2(sin3, -sqrt(1-sin3*sin3));
    if (abs(theta31-ref_joint.q[2]) < abs(theta32-ref_joint.q[2])) {
        ref_joint.q[2] = theta31;
    }
    else {
        ref_joint.q[2] = theta32;
    }

    //theta 2
    float theta23 = atan2((l2*cos(ref_joint.q[2])+(cos(ref_joint.q[0])*control_x+sin(ref_joint.q[0])*control_y)*(l3-l2*cos(ref_joint.q[2]))),
    (l3-l2*cos(ref_joint.q[2]))*control_z + l2*cos(ref_joint.q[2])*(cos(ref_joint.q[0])*control_x+sin(ref_joint.q[0])*control_y));
    ref_joint.q[1] = theta23 - ref_joint.q[2];

    //theta 4
    ref_joint.q[3] = atan2(sin(ref_pose.pitch)*sin(ref_joint.q[0]) + cos(ref_pose.pitch)*cos(ref_joint.q[0])*sin(ref_pose.roll),
        cos(ref_pose.pitch)*cos(ref_pose.roll)*cos(ref_joint.q[1])*sin(ref_joint.q[2]) + cos(ref_pose.pitch)*cos(ref_pose.roll)*cos(ref_joint.q[2])*sin(ref_joint.q[1]) + cos(ref_joint.q[0])*cos(ref_joint.q[1])*cos(ref_joint.q[1])*sin(ref_pose.pitch) - cos(ref_joint.q[0])*sin(ref_pose.pitch)*sin(ref_joint.q[1])*sin(ref_joint.q[2]) - cos(ref_pose.pitch)*cos(ref_joint.q[1])*cos(ref_joint.q[2])*sin(ref_pose.roll)*sin(ref_joint.q[0]) + cos(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[2]));

    //theta 5
    ref_joint.q[4] = atan2(sin(ref_pose.pitch)*sin(ref_joint.q[0])*sin(ref_joint.q[3]) + cos(ref_pose.pitch)*cos(ref_joint.q[0])*sin(ref_pose.roll)*sin(ref_joint.q[3]) + cos(ref_pose.pitch)*cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[3])*sin(ref_joint.q[2]) + cos(ref_pose.pitch)*cos(ref_pose.roll)*cos(ref_joint.q[2])*cos(ref_joint.q[3])*sin(ref_joint.q[1]) + cos(ref_joint.q[0])*cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_joint.q[3])*sin(ref_pose.pitch) - cos(ref_joint.q[0])*cos(ref_joint.q[3])*sin(ref_pose.pitch)*sin(ref_joint.q[1])*sin(ref_joint.q[2]) - cos(ref_pose.pitch)*cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_joint.q[3])*sin(ref_pose.roll)*sin(ref_joint.q[0]) + cos(ref_pose.pitch)*cos(ref_joint.q[3])*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[2]), cos(ref_pose.pitch)*cos(ref_pose.roll)*sin(ref_joint.q[1])*sin(ref_joint.q[2]) - cos(ref_pose.pitch)*cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[2]) + cos(ref_joint.q[0])*cos(ref_joint.q[1])*sin(ref_pose.pitch)*sin(ref_joint.q[2]) + cos(ref_joint.q[0])*cos(ref_joint.q[2])*sin(ref_pose.pitch)*sin(ref_joint.q[1]) - cos(ref_pose.pitch)*cos(ref_joint.q[1])*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[2]) - cos(ref_pose.pitch)*cos(ref_joint.q[2])*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1]));

    //theta 6
    ref_joint.q[5] = atan2(cos(ref_pose.pitch)*cos(ref_joint.q[3])*cos(ref_pose.yaw)*sin(ref_joint.q[0]) - cos(ref_pose.roll)*cos(ref_joint.q[0])*cos(ref_joint.q[3])*sin(ref_pose.yaw) - cos(ref_joint.q[0])*cos(ref_joint.q[3])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll) - cos(ref_joint.q[1])*sin(ref_pose.roll)*sin(ref_joint.q[2])*sin(ref_joint.q[3])*sin(ref_pose.yaw) - cos(ref_joint.q[2])*sin(ref_pose.roll)*sin(ref_joint.q[1])*sin(ref_joint.q[3])*sin(ref_pose.yaw) - cos(ref_pose.pitch)*cos(ref_joint.q[0])*cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_joint.q[3]) + cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[2])*sin(ref_joint.q[3]) + cos(ref_pose.roll)*cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[1])*sin(ref_joint.q[3]) + cos(ref_pose.pitch)*cos(ref_joint.q[0])*cos(ref_pose.yaw)*sin(ref_joint.q[1])*sin(ref_joint.q[2])*sin(ref_joint.q[3]) - cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[2])*sin(ref_joint.q[0])*sin(ref_joint.q[3])*sin(ref_pose.yaw) + cos(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[2])*sin(ref_joint.q[3])*sin(ref_pose.yaw) - cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[3]) + cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[2])*sin(ref_joint.q[3]), cos(ref_pose.pitch)*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_joint.q[0])*sin(ref_joint.q[3]) - cos(ref_pose.roll)*cos(ref_joint.q[0])*cos(ref_joint.q[4])*sin(ref_joint.q[3])*sin(ref_pose.yaw) + cos(ref_joint.q[1])*cos(ref_joint.q[2])*sin(ref_pose.roll)*sin(ref_joint.q[4])*sin(ref_pose.yaw) - sin(ref_pose.roll)*sin(ref_joint.q[1])*sin(ref_joint.q[2])*sin(ref_joint.q[4])*sin(ref_pose.yaw) - cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[4]) - cos(ref_pose.pitch)*cos(ref_joint.q[0])*cos(ref_joint.q[1])*cos(ref_pose.yaw)*sin(ref_joint.q[2])*sin(ref_joint.q[4]) - cos(ref_pose.pitch)*cos(ref_joint.q[0])*cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_joint.q[1])*sin(ref_joint.q[4]) - cos(ref_joint.q[0])*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[3]) + cos(ref_joint.q[1])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*sin(ref_pose.roll)*sin(ref_joint.q[2])*sin(ref_pose.yaw) + cos(ref_joint.q[2])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*sin(ref_pose.roll)*sin(ref_joint.q[1])*sin(ref_pose.yaw) + cos(ref_pose.roll)*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[1])*sin(ref_joint.q[2])*sin(ref_joint.q[4]) - cos(ref_pose.roll)*cos(ref_joint.q[1])*sin(ref_joint.q[0])*sin(ref_joint.q[2])*sin(ref_joint.q[4])*sin(ref_pose.yaw) - cos(ref_pose.roll)*cos(ref_joint.q[2])*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[4])*sin(ref_pose.yaw) + cos(ref_pose.pitch)*cos(ref_joint.q[0])*cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*cos(ref_pose.yaw) - cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[2]) - cos(ref_pose.roll)*cos(ref_joint.q[2])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_joint.q[1]) - cos(ref_pose.pitch)*cos(ref_joint.q[0])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_joint.q[1])*sin(ref_joint.q[2]) + cos(ref_pose.roll)*cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*sin(ref_joint.q[0])*sin(ref_pose.yaw) - cos(ref_pose.roll)*cos(ref_joint.q[3])*cos(ref_joint.q[4])*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[2])*sin(ref_pose.yaw) - cos(ref_joint.q[1])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[2])*sin(ref_joint.q[4]) - cos(ref_joint.q[2])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[4]) + cos(ref_joint.q[1])*cos(ref_joint.q[2])*cos(ref_joint.q[3])*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0]) - cos(ref_joint.q[3])*cos(ref_joint.q[4])*cos(ref_pose.yaw)*sin(ref_pose.pitch)*sin(ref_pose.roll)*sin(ref_joint.q[0])*sin(ref_joint.q[1])*sin(ref_joint.q[2]));

}

#define ANGLE_INCREMENT 0.1
#define PITCH_YAW_INCREMENT 0.01
extern RC rc;
void Arm::updateRefPose() {
    if (rc.switch_.l == 0) {
        ref_pose.x += rc.channel_.r_col * ANGLE_INCREMENT;
        ref_pose.y -= rc.channel_.r_row * ANGLE_INCREMENT;
        ref_pose.z += rc.channel_.l_col * ANGLE_INCREMENT;
    } else if (rc.switch_.l == 1) {
        ref_pose.pitch += rc.channel_.r_col * PITCH_YAW_INCREMENT;
        ref_pose.yaw += rc.channel_.r_row * PITCH_YAW_INCREMENT;
        ref_pose.roll += rc.channel_.l_row * PITCH_YAW_INCREMENT;
    }
}

Joint ref_joint;
void Arm::handle() {

    updateRefPose();

    //由末端姿态得到1-6角度

    ikine(ref_pose, ref_joint);

    //轨迹插值
    // get_joint();
    // Trajectory ref_trajectory(arm_joint, ref_joint);
    // ref_trajectory.handle();

    //设置角度

    for(int i=0; i<6; i++) ref_joint.q[i]*=57.3;

    m1.setAngle(ref_joint.q[0],0);
    m2.setAngle(-ref_joint.q[1],0);
    m3.setAngle(-ref_joint.q[2],0);
    m4.setAngle(ref_joint.q[3],0);
    m5.setAngle(-ref_joint.q[4],0);
    m6.setAngle(ref_joint.q[5],0);
}

