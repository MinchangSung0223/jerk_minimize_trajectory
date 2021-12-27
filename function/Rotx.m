function T = Rotx(theta)
T = eye(4);
R = [1 0 0;...
     0 cos(theta) -sin(theta);...
     0 sin(theta) cos(theta)];
T(1:3,1:3) = R;
end