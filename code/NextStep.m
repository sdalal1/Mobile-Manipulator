 function config = NextStep(curr_config, wheel_speed, dt, max_ang_s)
% Takes  curr_config : 12 vector current configuration
%        wheel_speed : 9 vector of joint speeds
%        dt: timestep
%        max_ang_s: Maximum speed allowed for motion
% Returns  config  : 12 vector after timstep dt
%
% Example Input:
% curr_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% wheel_speed = [10.0,10.0,10.0,10.0,0.0,0.0,0.0,0.0,0.0]';
% dt = 0.01;
% max_ang_s =10;
% time = 1;
% iter = time/dt;
% 
% for i = 1:iter
%     xx(i,:) = [curr_config(1,:),0];
%     % writematrix(curr_config,'move.csv')
%     curr_config = NextStep(curr_config, wheel_speed, dt, max_ang_s)';
% end

chassis_config = curr_config(1:3);
arm_config = curr_config(4:8);
wheel_angle = curr_config(9:12);
phi = 0;
for i = 1:length(wheel_speed)
    if wheel_speed(i) > max_ang_s
        wheel_speed(i) = max_ang_s;
    elseif wheel_speed(i) < -max_ang_s
        wheel_speed(i) = -max_ang_s;
    end
end


    l = 0.47/2;
    w = 0.3/2;
    r = 0.0475;		


theta_dot = wheel_speed(5:9)';
u = wheel_speed(1:4)';

new_arm_config(:,1) = arm_config + (theta_dot * dt);
new_wheel_config(:,1) = wheel_angle + (u * dt);

x = [l; l; -l; -l];
y = [w; -w; -w; w];
gamma = [-pi/4; pi/4; -pi/4; pi/4];
beta = zeros(4,1);
r_v = r*ones(4,1);

H = [ x.*sin(beta+gamma) - y.*cos(beta+gamma) , cos(beta+gamma+phi), sin(beta+gamma+phi)]./(r_v.*cos(gamma));

twist = pinv(H);


% twist = r/4 * ([[-1/(l + w),1/(l + w),1/(l + w),-1/(l + w)]; [1,1,1,1];[-1,1,-1,1]]);
twist_new = twist *(u*dt)';

w_bz = twist_new(1);
v_bx = twist_new(2);
v_by = twist_new(3);

    if w_bz < 0.0001
        qb = [0;v_bx;v_by];
    else
        qb = [w_bz;((v_bx * sin(w_bz)) + (v_by * (cos(w_bz) - 1))/w_bz);((v_by * sin(w_bz)) + (v_bx * (1 - cos(w_bz)))/w_bz)];
    end
T_sb = [[1,0,0];[0,cos(curr_config(1)),-sin(curr_config(1))];[0,sin(curr_config(1)),cos(curr_config(1))]];
del_chas = T_sb*qb;

new_chassis_config = chassis_config+del_chas';

config = cat(1,new_chassis_config',new_arm_config,new_wheel_config);
end

