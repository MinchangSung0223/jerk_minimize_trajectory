function T = Rotz(theta)
T = eye(4);
R = [cos(theta) -sin(theta) 0; ...
     sin(theta)  cos(theta) 0; ...
      0         0           1];
T(1:3,1:3) = R;
end