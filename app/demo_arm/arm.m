clc;
close all;


L1 = Link('revolute', "d", 0, "a", 0, "alpha", 0, 'modified');
L2 = Link('revolute', "d", 0, "a", 0, "alpha", -pi/2, 'modified');
L3 = Link('revolute', "d", 0, "a", 0.266, "alpha", 0, 'modified');
L4 = Link('revolute', "d", 0.28, "a", 0, "alpha", -pi/2, 'modified');
L5 = Link('revolute', "d", 0, "a", 0, "alpha", pi/2, 'modified');
L6 = Link('revolute', "d", 0.01, "a", 0, "alpha", -pi/2, 'modified');


Arm = SerialLink([L1,L2,L3,L4,L5,L6]);

% Arm.plot([0,0,0,0,0,0])
% Arm.teach()



%{

% 将 SE3 对象转换为齐次变换矩阵
T_matrix = double(T);

% 提取第3列的前3个元素
ez6 = T_matrix(1:3, 3);

% 提取第4列的前3个元素
o6 = T_matrix(1:3, 4);

o5 = o6 - ez6*0.01;

dist5 = o5(1)^2 +o5(2)^2 + o5(3)^2
maxdist = 0.28^2 +0.266^2
%}

%{
%使用蒙特卡洛法绘制机械臂的工作空间
N=15000;
theta1 = (360 * rand(N,1));
theta2 = (360 * rand(N,1));
theta3 = (360 * rand(N,1));
theta4 = (360 * rand(N,1));
theta5 = (360 * rand(N,1));
theta6 = (360 * rand(N,1));
 
for n = 1:N
    theta = [theta1(n),theta2(n),theta3(n),theta4(n),theta5(n),theta6(n)];
    workspace = Arm.fkine(theta);
    plot3(workspace.t(1),workspace.t(2),workspace.t(3),'b.','markersize',1);
    hold on;
end
Arm.plot(theta);  %动画显示
%进行机器人的实时展示
%}


syms theta1 theta2 theta3 theta4 theta5 theta6
syms a3 d4 d6
syms x y z yaw pitch roll

T01 = [
    cos(theta1), -sin(theta1), 0, 0;
    sin(theta1), cos(theta1), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1
];

T12 = [
    cos(theta2), -sin(theta2), 0, 0;
    0, 0, 1, 0;
    -sin(theta2), -cos(theta2), 0, 0;
    0, 0, 0, 1
];

T23 = [
    cos(theta3), -sin(theta3), 0, a3;
    sin(theta3), cos(theta3), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1
];

T34 = [
    cos(theta4), -sin(theta4), 0, 0;
    0, 0, 1, d4;
    -sin(theta4), -cos(theta4), 0, 0;
    0, 0, 0, 1
];

T45 = [
    cos(theta5), -sin(theta5), 0, 0;
    0, 0, -1, 0;
    sin(theta5), cos(theta5), 0, 0;
    0, 0, 0, 1
];

T56 = [
    cos(theta6), -sin(theta6), 0, 0;
    0, 0, 1, d6;
    -sin(theta6), -cos(theta6), 0, 0;
    0, 0, 0, 1
];

pose = transl(x,y,z) * trotz(yaw) * troty(pitch) * trotx(roll)
simplify(T01*T12*T23*T34*T45);


%用pose5 解算q1,q2,q3
%解算theta1
T15_r = simplify(T12*T23*T34*T45);
T15_l = simplify(inv(T01)*pose);
T15_r(2,4);
T15_l(2,4);
theta1 = atan2(y,x);

%解算theta3
T15_r(1,4);
T15_r(3,4);
T15_l(1,4);
T15_l(3,4);
simplify(a3*a3 + d4*d4 - T15_r(1,4)*T15_r(1,4) - T15_r(3,4)*T15_r(3,4));
sin3 = (a3*a3 + d4*d4 - (x*cos(theta1) + y*sin(theta1))*(x*cos(theta1) + y*sin(theta1)) - z*z)/(2*a3*d4);  %theta3两个解，由theta2必须是负的，舍去一个解

%解算theta2
T25_r = simplify(T23*T34*T45);
T25_l = simplify(inv(T12)*inv(T01)*pose);
T25_r(1,4);
T25_r(2,4);
T25_l(1,4);
T25_l(2,4);
theta2 = atan2(-(d4*cos(theta3)*(x*cos(theta1) + y*sin(theta1)) + z*(a3 - d4*sin(theta3))), ...
    ((a3 - d4*sin(theta3))*(x*cos(theta1) + y*sin(theta1)) - z*d4*cos(theta3)));



%用pose6解算q4,q5,q6
%解算theta6
T36_r = simplify(T34*T45*T56);
T36_l = simplify(inv(T23)*inv(T12)*inv(T01)*pose);
T36_(2,3) = simplify(T36_l(2,3));

T36_r(2,1);
T36_r(2,2);
T36_l(2,1);
T36_l(2,2);

theta6 = atan2(-(cos(pitch)*sin(roll)*sin(theta2)*sin(theta3) - cos(pitch)*cos(theta2)*cos(theta3)*sin(roll) + cos(roll)*cos(theta1)*cos(theta2)*sin(theta3)*sin(yaw) + cos(roll)*cos(theta1)*cos(theta3)*sin(theta2)*sin(yaw) - cos(roll)*cos(theta2)*cos(yaw)*sin(theta1)*sin(theta3) - cos(roll)*cos(theta3)*cos(yaw)*sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*cos(yaw)*sin(pitch)*sin(roll)*sin(theta3) - cos(theta1)*cos(theta3)*cos(yaw)*sin(pitch)*sin(roll)*sin(theta2) - cos(theta2)*sin(pitch)*sin(roll)*sin(theta1)*sin(theta3)*sin(yaw) - cos(theta3)*sin(pitch)*sin(roll)*sin(theta1)*sin(theta2)*sin(yaw)), ...
    (cos(theta2)*cos(theta3)*sin(pitch) - sin(pitch)*sin(theta2)*sin(theta3) - cos(pitch)*cos(theta1)*cos(theta2)*cos(yaw)*sin(theta3) - cos(pitch)*cos(theta1)*cos(theta3)*cos(yaw)*sin(theta2) - cos(pitch)*cos(theta2)*sin(theta1)*sin(theta3)*sin(yaw) - cos(pitch)*cos(theta3)*sin(theta1)*sin(theta2)*sin(yaw)));

%解算theta4
T36_r(1,3);
T36_r(3,3);
T36_l(1,3);
T36_l(3,3);

theta4 = atan2((cos(roll)*cos(theta1)*sin(pitch)*sin(yaw) - cos(theta1)*cos(yaw)*sin(roll) - sin(roll)*sin(theta1)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)*sin(theta1)), ...
    -(cos(theta1)*cos(theta2)*cos(theta3)*sin(roll)*sin(yaw) - cos(pitch)*cos(roll)*cos(theta3)*sin(theta2) - cos(pitch)*cos(roll)*cos(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*cos(yaw)*sin(roll)*sin(theta1) - cos(theta1)*sin(roll)*sin(theta2)*sin(theta3)*sin(yaw) + cos(yaw)*sin(roll)*sin(theta1)*sin(theta2)*sin(theta3) + cos(roll)*cos(theta1)*cos(theta2)*cos(theta3)*cos(yaw)*sin(pitch) - cos(roll)*cos(theta1)*cos(yaw)*sin(pitch)*sin(theta2)*sin(theta3) + cos(roll)*cos(theta2)*cos(theta3)*sin(pitch)*sin(theta1)*sin(yaw) - cos(roll)*sin(pitch)*sin(theta1)*sin(theta2)*sin(theta3)*sin(yaw)));


%解算theta5
T36_r(2,3);
T36_r(3,3);
T36_l(2,3);
T36_l(3,3);

theta5 = atan2((cos(roll)*cos(theta1)*sin(pitch)*sin(yaw) - cos(theta1)*cos(yaw)*sin(roll) - sin(roll)*sin(theta1)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch)*sin(theta1))/(sin(theta4)), ...
    (cos(pitch)*cos(roll)*sin(theta2)*sin(theta3) - cos(pitch)*cos(roll)*cos(theta2)*cos(theta3) - cos(theta1)*cos(theta2)*sin(roll)*sin(theta3)*sin(yaw) - cos(theta1)*cos(theta3)*sin(roll)*sin(theta2)*sin(yaw) + cos(theta2)*cos(yaw)*sin(roll)*sin(theta1)*sin(theta3) + cos(theta3)*cos(yaw)*sin(roll)*sin(theta1)*sin(theta2) - cos(roll)*cos(theta1)*cos(theta2)*cos(yaw)*sin(pitch)*sin(theta3) - cos(roll)*cos(theta1)*cos(theta3)*cos(yaw)*sin(pitch)*sin(theta2) - cos(roll)*cos(theta2)*sin(pitch)*sin(theta1)*sin(theta3)*sin(yaw) - cos(roll)*cos(theta3)*sin(pitch)*sin(theta1)*sin(theta2)*sin(yaw)));

