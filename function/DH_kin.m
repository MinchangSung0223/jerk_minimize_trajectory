function T = DH_kin(thetalist,alphalist,alist,dlist,i)

if (i==2) || (i==3)
   theta = thetalist(i)+pi/2;
elseif (i==4)
   theta = thetalist(i)+pi;
elseif (i==1)
   theta = thetalist(i)
else
    theta = thetalist(i)
end
alpha = alphalist(i);
a = alist(i);
d = dlist(i);
T = [cos(theta) -sin(theta) 0 a; ...
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;...
     sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;...
     0 0 0 1];
end