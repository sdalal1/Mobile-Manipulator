function xx = TrajectoryGenerator(X1, X2, X3, X4, X5,X6,X7,X8, X9,k)

% Takes X1-9: The 9 segments of transformation trajectories,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
% Returns xx: A list of discretized trajectory as a list of N * 9 matrices in SE(3)
%               separated in time by Tf/(N-1). The first in the list is 
%               X1-X2 and the last is X8-X9.
%
% Example Input:
% X1 = [[1, 0, 0, 0.2932]; [0, 1, 0, 0.094]; [0, 0, 1, 0.8475]; [0, 0, 0, 1]];
% X2 = [[-0.707, 0, 0.707, 1.0]; [0, 1, 0, 0.0]; [-0.707, 0, -0.707, 0.1]; [0, 0, 0, 1]];
% X3 = [[-0.707, 0, 0.707, 1.0]; [0, 1, 0, 0.0]; [-0.707, 0, -0.707, 0.025]; [0, 0, 0, 1]];
% X4 = [[-0.707, 0, 0.707, 1.0]; [0, 1, 0, 0.0]; [-0.707, 0, -0.707, 0.025]; [0, 0, 0, 1]];
% X5 = X2;
% X6 = [[0, 1, 0, 0.0]; [0.707, 0, -0.707, -1.0]; [-0.707,0, -0.707, 0.1]; [0, 0, 0, 1]];
% X7 = [[0, 1, 0, 0.0]; [0.707, 0, -0.707, -1.0]; [-0.707,0, -0.707, 0.025]; [0, 0, 0, 1]];
% X8 = [[0, 1, 0, 0.0]; [0.707,0, -0.707, -1.0]; [-0.707,0, -0.707, 0.025]; [0, 0, 0, 1]];
% X9 = X6;
% 
% traj = TrajectoryGenerator(X1, X2, X3, X4, X5,X6,X7,X8, X9,1);
% writematrix(traj,'traj.csv')

method =5; %The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling and 5 indicates quintic (fifth-order polynomial) time scaling.
% traj1 = CartesianTrajectory(X1, X2, 5, 500*k, method); %initial to cube intial standoff
% traj2 = CartesianTrajectory(X2, X3, 4.5, 450*k, method); % cube initial standoff to cube location
% traj3 = CartesianTrajectory(X3, X4, 1, 100*k, method); % cube gripping
% traj4 = CartesianTrajectory(X4, X5, 4.5, 450*k, method); % cube gripping to cube standoff
% traj5 = CartesianTrajectory(X5, X6, 5, 500*k, method); % cube initial standoff to final stanoff
% traj6 = CartesianTrajectory(X6, X7, 4.5, 450*k, method); % cube final standoff to cube final pos
% traj7 = CartesianTrajectory(X7, X8, 1, 100*k, method); % cube ungripping
% traj8 = CartesianTrajectory(X8, X9, 4.5, 450*k, method); % cube drop to final standoff

traj1 = ScrewTrajectory(X1, X2, 5, 500*k, method); %initial to cube intial standoff
traj2 = ScrewTrajectory(X2, X3, 4.5, 450*k, method); % cube initial standoff to cube location
traj3 = ScrewTrajectory(X3, X4, 1, 100*k, method); % cube gripping
traj4 = ScrewTrajectory(X4, X5, 4.5, 450*k, method); % cube gripping to cube standoff
traj5 = ScrewTrajectory(X5, X6, 5, 500*k, method); % cube initial standoff to final stanoff
traj6 = ScrewTrajectory(X6, X7, 4.5, 450*k, method); % cube final standoff to cube final pos
traj7 = ScrewTrajectory(X7, X8, 1, 100*k, method); % cube ungripping
traj8 = ScrewTrajectory(X8, X9, 4.5, 450*k, method); % cube drop to final standoff


for i = 1: length(traj1)
    xx1(i,:) = [(traj1{i}(1,1)),(traj1{i}(1,2)),(traj1{i}(1,3)), (traj1{i}(2,1)), (traj1{i}(2,2)), (traj1{i}(2,3)), (traj1{i}(3,1)), (traj1{i}(3,2)), (traj1{i}(3,3)), (traj1{i}(1,4)), (traj1{i}(2,4)), (traj1{i}(3,4)),0];
end
for i = 1: length(traj2)
    xx2(i,:) = [ (traj2{i}(1,1)), (traj2{i}(1,2)), (traj2{i}(1,3)), (traj2{i}(2,1)), (traj2{i}(2,2)), (traj2{i}(2,3)), (traj2{i}(3,1)), (traj2{i}(3,2)), (traj2{i}(3,3)), (traj2{i}(1,4)), (traj2{i}(2,4)), (traj2{i}(3,4)),0];
end
for i = 1: length(traj3)
    xx3(i,:) = [ (traj3{i}(1,1)), (traj3{i}(1,2)), (traj3{i}(1,3)), (traj3{i}(2,1)), (traj3{i}(2,2)), (traj3{i}(2,3)), (traj3{i}(3,1)), (traj3{i}(3,2)), (traj3{i}(3,3)), (traj3{i}(1,4)), (traj3{i}(2,4)), (traj3{i}(3,4)),1];
end
for i = 1: length(traj4)
    xx4(i,:) = [ (traj4{i}(1,1)), (traj4{i}(1,2)), (traj4{i}(1,3)), (traj4{i}(2,1)), (traj4{i}(2,2)), (traj4{i}(2,3)), (traj4{i}(3,1)), (traj4{i}(3,2)), (traj4{i}(3,3)), (traj4{i}(1,4)), (traj4{i}(2,4)), (traj4{i}(3,4)),1];
end
for i = 1: length(traj5)
    xx5(i,:) = [ (traj5{i}(1,1)), (traj5{i}(1,2)), (traj5{i}(1,3)), (traj5{i}(2,1)), (traj5{i}(2,2)), (traj5{i}(2,3)), (traj5{i}(3,1)), (traj5{i}(3,2)), (traj5{i}(3,3)), (traj5{i}(1,4)), (traj5{i}(2,4)), (traj5{i}(3,4)),1];
end
for i = 1: length(traj6)
    xx6(i,:) = [ (traj6{i}(1,1)), (traj6{i}(1,2)), (traj6{i}(1,3)), (traj6{i}(2,1)), (traj6{i}(2,2)), (traj6{i}(2,3)), (traj6{i}(3,1)), (traj6{i}(3,2)), (traj6{i}(3,3)), (traj6{i}(1,4)), (traj6{i}(2,4)), (traj6{i}(3,4)),1];
end
for i = 1: length(traj7)
    xx7(i,:) = [ (traj7{i}(1,1)), (traj7{i}(1,2)), (traj7{i}(1,3)), (traj7{i}(2,1)), (traj7{i}(2,2)), (traj7{i}(2,3)), (traj7{i}(3,1)), (traj7{i}(3,2)), (traj7{i}(3,3)), (traj7{i}(1,4)), (traj7{i}(2,4)), (traj7{i}(3,4)),0];
end
for i = 1: length(traj8)
    xx8(i,:) = [ (traj8{i}(1,1)), (traj8{i}(1,2)), (traj8{i}(1,3)), (traj8{i}(2,1)), (traj8{i}(2,2)), (traj8{i}(2,3)), (traj8{i}(3,1)), (traj8{i}(3,2)), (traj8{i}(3,3)), (traj8{i}(1,4)), (traj8{i}(2,4)), (traj8{i}(3,4)),0];
end

xx = cat(1,xx1,xx2,xx3,xx4,xx5,xx6,xx7,xx8);
% writematrix(xx,'traj.csv')