% 参数定义 (保持不变)
m = 1.0; g = 9.81; l = 0.25;
Ixx = 0.01; Iyy = 0.01; Izz = 0.02;

% PID 控制器参数 (调整了z轴参数)
Kp_pos = [1.0, 1.0, 2.0]; 
Ki_pos = [0.1, 0.1, 0.2];
Kd_pos = [0.2, 0.2, 0.5];

Kp_att = [8.0, 8.0, 2.0]; % 增加姿态控制增益
Ki_att = [0.1, 0.1, 0.1];
Kd_att = [0.5, 0.5, 0.5];

% 初始化变量
x = 0; y = 0; z = 0;
vx = 0; vy = 0; vz = 0;
phi = 0; theta = 0; psi = 0;
p = 0; q = 0; r = 0;

% 目标位置
x_des = 1.0; y_des = 1.0; z_des = 2.0;

% 时间设置
dt = 0.01; T = 10;
time = 0:dt:T;

% 存储数据
pos_history = zeros(length(time), 3);
att_history = zeros(length(time), 3);

% 初始化控制器状态
sum_ex = 0; sum_ey = 0; sum_ez = 0;
pre_ex = x_des - x; pre_ey = y_des - y; pre_ez = z_des - z;
sum_e_phi = 0; sum_e_theta = 0; sum_e_psi = 0; % 新增姿态积分项

% 主循环
for i = 1:length(time)
    % ===== 外环位置控制 =====
    ex = x_des - x;
    ey = y_des - y;
    ez = z_des - z;
    
    sum_ex = sum_ex + ex;
    sum_ey = sum_ey + ey;
    sum_ez = sum_ez + ez;
    
    d_ex = (ex - pre_ex)/dt;
    d_ey = (ey - pre_ey)/dt;
    d_ez = (ez - pre_ez)/dt;
    
    pre_ex = ex;
    pre_ey = ey;
    pre_ez = ez;
    
    % 位置PID输出为加速度指令
    ax_des = Kp_pos(1)*ex + Ki_pos(1)*sum_ex*dt + Kd_pos(1)*d_ex;
    ay_des = Kp_pos(2)*ey + Ki_pos(2)*sum_ey*dt + Kd_pos(2)*d_ey;
    az_des = Kp_pos(3)*ez + Ki_pos(3)*sum_ez*dt + Kd_pos(3)*d_ez;
    
    % ===== 计算期望姿态和总推力 =====
    T_des = m*(g - az_des); % 总推力计算
    phi_des = atan2(ay_des, g);   % 横滚角指令
    theta_des = atan2(-ax_des, g); % 俯仰角指令
    
    % ===== 内环姿态控制 =====
    e_phi = phi_des - phi;
    e_theta = theta_des - theta;
    e_psi = 0 - psi;
    
    sum_e_phi = sum_e_phi + e_phi*dt;
    sum_e_theta = sum_e_theta + e_theta*dt;
    sum_e_psi = sum_e_psi + e_psi*dt;
    
    p_des = Kp_att(1)*e_phi + Ki_att(1)*sum_e_phi + Kd_att(1)*(-p);
    q_des = Kp_att(2)*e_theta + Ki_att(2)*sum_e_theta + Kd_att(2)*(-q);
    r_des = Kp_att(3)*e_psi + Ki_att(3)*sum_e_psi + Kd_att(3)*(-r);
    
    % ===== 更新动力学 =====
    [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] = update_dynamics(...
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, ...
        p_des, q_des, r_des, T_des, dt, m, g, l, Ixx, Iyy, Izz);
    
    % 保存数据
    pos_history(i, :) = [x, y, z];
    att_history(i, :) = [phi, theta, psi];
end

% 绘图 (保持不变)
figure;
subplot(2,1,1);
plot(time, pos_history(:,1), title('X Position')); 
subplot(2,1,2);
plot(time, att_history(:,1)), title('Roll Angle');

% ===== 修正后的动力学函数 =====
function [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] = update_dynamics(...
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r, ...
        p_des, q_des, r_des, T_des, dt, m, g, l, Ixx, Iyy, Izz)
    
    % 角速度一阶响应
    p = p + dt*10*(p_des - p);
    q = q + dt*10*(q_des - q);
    r = r + dt*10*(r_des - r);
    
    % 欧拉角更新 (修正公式)
    phi = phi + dt*(p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta));
    theta = theta + dt*(q*cos(phi) - r*sin(phi));
    psi = psi + dt*(q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta));
    
    % ===== 关键修正：正确的推力计算 =====
    % 旋转矩阵 (机体到惯性系)
    R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
         -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)];
    
    % 机体坐标系中的推力向量 [0,0,-T_des]
    thrust_body = [0; 0; -T_des];
    
    % 转换到惯性坐标系
    thrust_inertial = R * thrust_body;
    
    % 计算加速度 (重力 + 推力)
    accel = [0; 0; g] + thrust_inertial/m;
    
    % 更新速度和位置
    vx = vx + dt*accel(1);
    vy = vy + dt*accel(2);
    vz = vz + dt*accel(3);
    
    x = x + dt*vx;
    y = y + dt*vy;
    z = z + dt*vz;
end