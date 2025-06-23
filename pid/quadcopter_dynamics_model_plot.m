ModelParam_c_T = 1.105e-05;    % 螺旋桨拉力系数
ModelParam_c_M = 1.779e-07*2;  % 螺旋桨力矩系数
ModelParam_d = 0.225;          % 机体中心和任一电机的距离(m)
ModelParam_m = 1.4;            % 四旋翼飞行器质量(kg)
ModelParam_g = 9.8;            % 重力加速度(m/s^2)
ModelParam_I_xx = 0.0211;      % 四旋翼x轴转动惯量(kg·m^2)
ModelParam_I_yy = 0.0219;      % 四旋翼y轴转动惯量(kg·m^2)
ModelParam_I_zz = 0.0366;      % 四旋翼z轴转动惯量(kg·m^2)
ModelParam_J_RP = 0.0001287;   % 整个电机转子和螺旋桨绕转轴的总转动惯量(kg·m^2)
ModelInit_Pos_z = -100;        % 四旋翼初始高度
input1 = 557.142;
input2 = 557.142;
input3 = 557.142;
input4 = 557.142;
sim("Quadcopter_Dynamics_Model");
plot3(simout.Data(:,1),simout.Data(:,2),simout.Data(:,3))

