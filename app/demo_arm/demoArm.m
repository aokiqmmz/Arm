%demo_arm建模

L1 = Link([0,0,0,0,0,0],'qlim',[-5*pi,5*pi],'modified');
L2 = Link([0,0,0,pi/2,0,0],'qlim',[-5*pi,5*pi],'modified');
L3 = Link([0,-5,-40,0,0,pi/2],'qlim',[-5*pi,5*pi],'modified');
L4 = Link([0,30,0,pi/2,0,-pi/2],'qlim',[-5*pi,5*pi],'modified');
L5 = Link([0,0,0,-pi/2,0,pi/2],'qlim',[-5*pi,5*pi],'modified');
L6 = Link([0,0,0,pi/2,0,0],'qlim',[-5*pi,5*pi],'modified');


Arm = SerialLink([L1,L2,L3,L4,L5,L6])


Arm.plot([0,0,0,0,0,0])
Arm.teach()



% 获得6个连杆的变换矩阵

%
% 定义符号变量

syms theta1 theta2 theta3 theta4 theta5 theta6
syms l1 l2 l3
syms x y z yaw pitch roll

% 
% % 定义每个连杆的MDH参数
% params = struct('theta', [theta1, theta2, theta3, theta4, theta5, theta6], ...
%                 'd', Arm.d, ...
%                 'a', Arm.a, ...
%                 'alpha', Arm.alpha);
% 
% % 创建每个连杆的变换矩阵
% T = cell(1, 6);
% for i = 1:6
%     T{i} = [cos(params.theta(i)), -sin(params.theta(i)), 0, params.a(i);
%             sin(params.theta(i))*cos(params.alpha(i)), cos(params.theta(i))*cos(params.alpha(i)), -sin(params.alpha(i)), -sin(params.alpha(i))*params.d(i);
%             sin(params.theta(i))*sin(params.alpha(i)), cos(params.theta(i))*sin(params.alpha(i)), cos(params.alpha(i)), cos(params.alpha(i))*params.d(i);
%             0, 0, 0, 1];
% end
% 
% T01 = simplify(T{1})
% T12 = simplify(T{2})
% T23 = simplify(T{3})
% T34 = simplify(T{4})
% T45 = simplify(T{5})
% T56 = simplify(T{6})
% 
%算出来的有误差，导致矩阵某些项不是0，不利后续分析
%例如T12 =
% 
% [                                                    cos(theta2),                                                    -sin(theta2),                                                 0, 0]
% [(4967757600021511*sin(theta2))/81129638414606681695789005144064, (4967757600021511*cos(theta2))/81129638414606681695789005144064,                                                -1, 0]
% [                                                    sin(theta2),                                                     cos(theta2), 4967757600021511/81129638414606681695789005144064, 0]
% [                                                              0,                                                               0,                                                 0, 1]
%尚未找到解决方法，仍采用手动化简为0
%
%

% 定义每个连杆的MDH参数
params = struct('theta', [theta1, theta2, theta3, theta4, theta5, theta6], ...
                'd', Arm.d, ...
                'a', Arm.a, ...
                'alpha', Arm.alpha);

% 创建每个连杆的变换矩阵
T = cell(1, 6);
for i = 1:6
    T{i} = [cos(params.theta(i)), -sin(params.theta(i)), 0, params.a(i);
            sin(params.theta(i))*cos(params.alpha(i)), cos(params.theta(i))*cos(params.alpha(i)), -sin(params.alpha(i)), -sin(params.alpha(i))*params.d(i);
            sin(params.theta(i))*sin(params.alpha(i)), cos(params.theta(i))*sin(params.alpha(i)), cos(params.alpha(i)), cos(params.alpha(i))*params.d(i);
            0, 0, 0, 1];
end


T01 = [
    cos(theta1), -sin(theta1), 0, 0;
    sin(theta1), cos(theta1), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1
];

T12 = [
    cos(theta2), -sin(theta2), 0, 0;
    0, 0, -1, 0;
    sin(theta2), cos(theta2), 0, 0;
    0, 0, 0, 1
];

T23 = [
    cos(theta3), -sin(theta3), 0, -l2;
    sin(theta3), cos(theta3), 0, 0;
    0, 0, 1, -l1;
    0, 0, 0, 1
];

T34 = [
    cos(theta4), -sin(theta4), 0, 0;
    0, 0, -1, -l3;
    sin(theta4), cos(theta4), 0, 0;
    0, 0, 0, 1
];

T45 = [
    cos(theta5), -sin(theta5), 0, 0;
    0, 0, 1, 0;
    -sin(theta5), -cos(theta5), 0, 0;
    0, 0, 0, 1
];

T56 = [
    cos(theta6), -sin(theta6), 0, 0;
    0, 0, -1, 0;
    sin(theta6), cos(theta6), 0, 0;
    0, 0, 0, 1
];

%末端位姿矩阵

pose = transl(x,y,z) * trotx(roll) * troty(pitch) * trotz(yaw);





%解算theta1
T16_r = simplify(T12*T23*T34*T45*T56);
T16_l = transpose(T01)*pose;
T16_r(2,4);
T16_l(2,4);
rou1 = sqrt(x*x + y*y);
%theta1 = atan2(y,x) - atan2(l1/rou, +-sqrt(1-(l1/rou1)*(l1/rou1))  %多解

%解算theta3
simplify(l2*l2 + l3*l3 -T16_r(1,4)*T16_r(1,4)-T16_r(3,4)*T16_r(3,4));
sin3 = simplify((l2*l2 + l3*l3 -T16_l(1,4)*T16_l(1,4)-T16_l(3,4)*T16_l(3,4))/(2*l2*l3));
%theta3 = atan2(sin3, +-sqrt(1-sin3*sin3)) %多解

%解算theta2
T36_r = simplify(T34*T45*T56);
T36_l = simplify(inv(T23)*inv(T12)*inv(T01)*pose);
simplify(T36_l(2,4));
simplify(T36_l(1,4));
theta23 = atan2((l2*cos(theta3)+(cos(theta1)*x+sin(theta1)*y)*(l3-l2*cos(theta3))), ...
    (l3-l2*cos(theta3))*z + l2*cos(theta3)*(cos(theta1)*x+sin(theta1)*y));
theta2 = theta23 - theta3;

%解算theta4
T36_r(1,3);
T36_r(3,3);
theta4 = atan2(T36_l(3,3), T36_l(1,3));

%解算theta5
T46_r = simplify(T45*T56)
T46_l = simplify(inv(T34)*inv(T23)*inv(T12)*inv(T01)*pose)

T46_r(1,3)
T46_r(3,3)
theta5 = atan2(T46_l(1,3), T46_l(3,3))

%解算theta6
T56_r = simplify(T56)
T56_l = simplify(inv(T45)*inv(T34)*inv(T23)*inv(T12)*inv(T01)*pose)

theta6 = atan2(T56_l(3,1), T56_l(1,1))