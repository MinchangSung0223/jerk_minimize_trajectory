function traj = JointJerkTrajectory(thetastart, thetaend, Tf, N, method)

timegap = Tf / (N - 1);
traj = zeros(size(thetastart, 1), N);
for i = 1: N
    if method == 3
        s = JerkCubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = JerkQuinticTimeScaling(Tf, timegap * (i - 1));
    end
    traj(:, i) = s * (thetaend - thetastart);
end
traj = traj';
end