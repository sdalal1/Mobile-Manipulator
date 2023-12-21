%Final Assignment
% 
X1 = [[0,0,1,0];[0,1,0,0];[-1,0,0,0.5];[0,0,0,1]];
X2 = [[-0.707, 0., 0.707, 1.]; [0., 1., 0., 0.0]; [-0.707, 0., -0.707, 0.1]; [0., 0., 0., 1.]];
X3 = [[-0.707, 0., 0.707, 1.]; [0., 1., 0., 0.0]; [-0.707, 0., -0.707, 0.025]; [0., 0., 0., 1.]];
X4 = [[-0.707, 0., 0.707, 1.]; [0., 1., 0., 0.0]; [-0.707, 0., -0.707, 0.025]; [0., 0., 0., 1.]];
X5 = X2;
X6 = [[0., 1., 0., 0.0]; [0.707, 0., -0.707, -1.0]; [-0.707,0., -0.707, 0.1]; [0., 0., 0., 1.]];
X7 = [[0., 1., 0., 0.0]; [0.707, 0., -0.707, -1.0]; [-0.707,0., -0.707, 0.025]; [0., 0., 0., 1.]];
X8 = [[0., 1., 0., 0.0]; [0.707,0., -0.707, -1.0]; [-0.707,0., -0.707, 0.025]; [0., 0., 0., 1.]];
X9 = X6;
% X1 = [[0,0,1,0.5];[0,1,0,1.2];[-1,0,0,0.5];[0,0,0,1]]; New_loc
% X2 = [[-0.707, 0., 0.707, 1.]; [0., 1., 0., 1.0]; [-0.707, 0., -0.707,
% 0.1]; [0., 0., 0., 1.]]; New_loc
% X3 = [[-0.707, 0., 0.707, 1.]; [0., 1., 0., 1.0]; [-0.707, 0., -0.707,
% 0.025]; [0., 0., 0., 1.]]; New_loc
% X4 = [[-0.707, 0., 0.707, 1.]; [0., 1., 0., 1.0]; [-0.707, 0., -0.707,
% 0.025]; [0., 0., 0., 1.]]; New_loc
% X5 = X2; New_loc
% X6 = [[0., 1., 0., 1.0]; [0.707, 0., -0.707, -1.0]; [-0.707,0., -0.707,
% 0.1]; [0., 0., 0., 1.]]; New_loc
% X7 = [[0., 1., 0., 1.0]; [0.707, 0., -0.707, -1.0]; [-0.707,0., -0.707,
% 0.025]; [0., 0., 0., 1.]]; New_loc
% X8 = [[0., 1., 0., 1.0]; [0.707,0., -0.707, -1.0]; [-0.707,0., -0.707,
% 0.025]; [0., 0., 0., 1.]]; New_loc
% X9 = X6; New_loc
k=1.0;
traj_desired = TrajectoryGenerator(X1, X2, X3, X4, X5,X6,X7,X8, X9,k);
config = zeros(12); %Overshoot and Best
% config = [0,0,0,0.0,0.0,-1.57,0,0,0,0,0,0]; Best config
% config = [0.5, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
% New config of box
max_ang_s = 10.0;
err =zeros(6,1);
dt = 0.01;

% K_p = 0.7 * eye(6,6); %Best Kp
% K_i = 0.001 * eye(6,6); %Best Ki
K_p = 3.5* eye(6,6); %overshoot Kp
K_i = 0.5* eye(6,6); %overshoot Ki
% K_p = 1.9* eye(6,6); %New loc Kp
% K_i = 0.001* eye(6,6); %New Loc Ki

for i=1:(length(traj_desired)-1)
    X_d = [[traj_desired(i,1),traj_desired(i,2),traj_desired(i,3),traj_desired(i,10)];
        [traj_desired(i,4),traj_desired(i,5),traj_desired(i,6),traj_desired(i,11)];
         [traj_desired(i,7),traj_desired(i,8),traj_desired(i,9),traj_desired(i,12)];
         [0.,0.,0.,1.]];
    X_d_next = [[traj_desired(i+1,1),traj_desired(i+1,2),traj_desired(i+1,3),traj_desired(i+1,10)];
        [traj_desired(i+1,4),traj_desired(i+1,5),traj_desired(i+1,6),traj_desired(i+1,11)];
        [traj_desired(i+1,7),traj_desired(i+1,8),traj_desired(i+1,9),traj_desired(i+1,12)];
        [0.,0.,0.,1.]];
    phi = config(1);
    Tsb = [[cos(phi) ,-sin(phi), 0. ,config(2)]; [sin(phi) ,cos(phi) ,0. ,config(3)]; [0., 0., 1. ,0.0963]; [0. ,0. ,0. ,1.]];
    Moe = [[1.,0.,0.,0.033];[0.,1.,0.,0.];[0.,0.,1.,0.6546];[0.,0.,0.,1.]];
    Blist = [[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0],[0;-1;0;-0.3526;0;0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]];
    Toe = FKinBody(Moe, Blist, config(4:8)');
    Tbo = [[1.,0.,0.,0.1662];[0.,1.,0.,0.];[0.,0.,1.,0.0026];[0.,0.,0.,1.]];
    X = Tsb*Tbo*Toe;
    [V,error] = FeedbackControl(X,X_d,X_d_next,K_p, K_i, dt,err,config);
    err = error+err;
    new_config = NextStep(config, round(V,4,"significant"), dt, max_ang_s);
    config = new_config';
    traj_act(i,:) = [config(1,:), traj_desired(i,13)];
    err_traj(i,:) = [error'];

end

disp("Generating animation csv file.")
writematrix(traj_act,'overshoot.csv')
writematrix(err_traj,'overshoot_error.csv')
% writematrix(traj_act,'best.csv')
% writematrix(err_traj,'best_error.csv')

% writematrix(traj_act,'new_loc.csv')
% writematrix(err_traj,'new_loc_err.csv')
err_traj(:,1);

disp("Writing error plot data.")
plot(err_traj(:,1))
title('Xerr vs iterations')
xlabel('Iterations') 
ylabel('Error Magnitude (m/s,rad/s)') 
hold on
plot(err_traj(:,2))
plot(err_traj(:,3))
plot(err_traj(:,4))
plot(err_traj(:,5))
plot(err_traj(:,6))
legend({'wx','wy','wz','vx','vy','vz'})
hold off

disp("Done.")
clear