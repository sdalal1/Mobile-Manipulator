function [V_e,Xerr] = FeedbackControl(X,X_d,X_d_next,K_p, K_i, dt,err,config)
% Takes X: Transformation matrix of currrent actual endeffector wrt space
%       X_d: The current end effector cnfiguration
%       X_d_next: The end effector configuration 
%       K_p: The proportional gain which is a 6,1 vector
%       K_i: The integral gain which is a 6,1 vector
%       dt: The timestep between two trajectories
%       err: The increasing error
%       config: the current configuration of the youbot
% Returns V_e: The commanded end effector twist 
%         Xerr: Xerr of every step
%
% Example Input:
% config = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0];
% X = [[0.170,0,0.985,0.387];[0,1,0,0];[-0.985,0,0.170,0.570];[0,0,0,1]];
% X_d = [[0,0,1,0.5];[0,1,0,0];[-1,0,0,0.5];[0,0,0,1]];
% X_d_next = [[0,0,1,0.6];[0,1,0,0];[-1,0,0,0.3];[0,0,0,1]];
% K_p = 1.0* eye(6,6);
% K_i = 0.0* eye(6,6);
% dt = 0.01;
% 
% [V_e,error] = FeedbackControl(X,X_d,X_d_next,K_p, K_i, dt,0,config)
    l = 0.475/2;
    w = 0.3/2;
    r = 0.0475;	
    % config = [0,0,0,0,0,0.2,-1.6,0];
    phi = 0;
    Tbo = [[1,0,0,0.1662];[0,1,0,0];[0,0,1,0.0026];[0,0,0,1]];
    Moe = [[1,0,0,0.033];[0,1,0,0];[0,0,1,0.6546];[0,0,0,1]];
    Blist = [[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0],[0;-1;0;-0.3526;0;0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]];
    arm_joints = config(4:8)';

    x = [l; l; -l; -l];
    y = [w; -w; -w; w];
    gamma = [-pi/4; pi/4; -pi/4; pi/4];
    beta = zeros(4,1);
    r_v = r*ones(4,1);

    H = [ x.*sin(beta+gamma) - y.*cos(beta+gamma) , cos(beta+gamma+phi), sin(beta+gamma+phi)]./(r_v.*cos(gamma));

    twist = pinv(H);

    F6 = [zeros(1,4);zeros(1,4);twist;zeros(1,4)];
    % F6 = r / 4 * [[0, 0, 0, 0];[0, 0, 0, 0];[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)];[1, 1, 1, 1];[-1, 1, -1, 1];[0, 0, 0, 0]];

    Toe = FKinBody(Moe,Blist,arm_joints);
    Toe_inv = TransInv(Toe);
    Tbo_inv = TransInv(Tbo);
    J_a = JacobianBody(Blist,arm_joints);
    J_b = Adjoint(Toe_inv*Tbo_inv)*F6;

    J_e = [J_b,J_a];
    ps_inv = pinv(J_e, 1e-4);

    X_inv = TransInv(X);
    Xerr = se3ToVec(MatrixLog6(X_inv*X_d));
    err_acc = (err+Xerr)*dt;
    Xd_inv = TransInv(X_d);
    V_d = se3ToVec(1/dt * (MatrixLog6(Xd_inv*X_d_next)));
    Adj_xxd = Adjoint(X_inv*X_d);

    % V = Adj_xxd*V_d + K_p*Xerr + K_i*error;
    V = Adj_xxd*V_d + K_p*Xerr + K_i*err_acc;

    V_e = ps_inv*V;

    
end

