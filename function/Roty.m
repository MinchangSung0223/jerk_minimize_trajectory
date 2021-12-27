function T = Roty(theta)
T = eye(4);
R = [cos(theta) 0 sin(theta); ...
     0          1         0 ; ...
     -sin(theta) 0   cos(theta)];
 T(1:3,1:3) = R;
end