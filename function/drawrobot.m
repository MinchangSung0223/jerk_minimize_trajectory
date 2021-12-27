function drawrobot(thetalist,show_val,fv0,fv1,fv2,fv3,fv4,fv5,fv6)
thetalist_temp = thetalist;
% thetalist_temp(2) = thetalist_temp(2) +pi/2;
% thetalist_temp(3) = thetalist_temp(3) +pi/2;
% thetalist_temp(4) = thetalist_temp(4) +pi;

 
 p0 = [0,0,0,1]';
 p0x = [0.1,0,0,1]';
 p0y = [0,0.1,0,1]';
 p0z = [0,0,0.1,1]';
 
T = {};
for i=1:1:length(thetalist)
T{i} = DH_kin(thetalist,[0 pi/2 0 pi/2 pi/2 -pi/2],[0 0 0.450 0 0 0 ],[0.300 0 0.0035 0.350 0.1835 0.228],i);
end
 p1 = T{1}*p0;
 p1x = T{1}*p0x;
 p1y = T{1}*p0y;
 p1z = T{1}*p0z;
 
 p2 = T{1}*T{2}*p0;
 p2x = T{1}*T{2}*p0x;
 p2y = T{1}*T{2}*p0y;
 p2z = T{1}*T{2}*p0z;
 
 p3 = T{1}*T{2}*T{3}*p0;
 p3x = T{1}*T{2}*T{3}*p0x;
 p3y = T{1}*T{2}*T{3}*p0y;
 p3z = T{1}*T{2}*T{3}*p0z;
 
 p4 = T{1}*T{2}*T{3}*T{4}*p0;
 p4x = T{1}*T{2}*T{3}*T{4}*p0x;
 p4y = T{1}*T{2}*T{3}*T{4}*p0y;
 p4z = T{1}*T{2}*T{3}*T{4}*p0z;
 
 p5 = T{1}*T{2}*T{3}*T{4}*T{5}*p0;
 p5x = T{1}*T{2}*T{3}*T{4}*T{5}*p0x;
 p5y = T{1}*T{2}*T{3}*T{4}*T{5}*p0y;
 p5z = T{1}*T{2}*T{3}*T{4}*T{5}*p0z;
 
 p6 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*p0;
 p6x = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*p0x;
 p6y = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*p0y;
 p6z = T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*p0z;
 

 
 plot3([0,2 0 0 0 -2 0 0 0 0 0 0],[0,0 0 2 0 0 0 -2 0 0 0 0],[0,0 0 0 0 0 0 0 0 0 0 2])
  hold on;
   pbaspect([1,1,1])
 grid on;
 line([p0(1) p1(1),p2(1),p3(1),p4(1),p5(1),p6(1)],[p0(2) p1(2),p2(2),p3(2),p4(2),p5(2),p6(2)],[p0(3) p1(3),p2(3),p3(3),p4(3),p5(3),p6(3)])
 plot3([p0(1) p1(1),p2(1),p3(1),p4(1),p5(1),p6(1)],[p0(2) p1(2),p2(2),p3(2),p4(2),p5(2),p6(2)],[p0(3) p1(3),p2(3),p3(3),p4(3),p5(3),p6(3)],'r.')
 
 line([p0(1),p0x(1)],[p0(2),p0x(2)],[p0(3),p0x(3)],'Color','red','LineWidth',2)
 line([p0(1),p0y(1)],[p0(2),p0y(2)],[p0(3),p0y(3)],'Color','green','LineWidth',2)
 line([p0(1),p0z(1)],[p0(2),p0z(2)],[p0(3),p0z(3)],'Color','blue','LineWidth',2)
 line([p1(1),p1x(1)],[p1(2),p1x(2)],[p1(3),p1x(3)],'Color','red','LineWidth',2)
 line([p1(1),p1y(1)],[p1(2),p1y(2)],[p1(3),p1y(3)],'Color','green','LineWidth',2)
 line([p1(1),p1z(1)],[p1(2),p1z(2)],[p1(3),p1z(3)],'Color','blue','LineWidth',2)
line([p2(1),p2x(1)],[p2(2),p2x(2)],[p2(3),p2x(3)],'Color','red','LineWidth',2)
 line([p2(1),p2y(1)],[p2(2),p2y(2)],[p2(3),p2y(3)],'Color','green','LineWidth',2)
 line([p2(1),p2z(1)],[p2(2),p2z(2)],[p2(3),p2z(3)],'Color','blue','LineWidth',2)
 line([p3(1),p3x(1)],[p3(2),p3x(2)],[p3(3),p3x(3)],'Color','red','LineWidth',2)
 line([p3(1),p3y(1)],[p3(2),p3y(2)],[p3(3),p3y(3)],'Color','green','LineWidth',2)
 line([p3(1),p3z(1)],[p3(2),p3z(2)],[p3(3),p3z(3)],'Color','blue','LineWidth',2)
 line([p4(1),p4x(1)],[p4(2),p4x(2)],[p4(3),p4x(3)],'Color','red','LineWidth',2)
 line([p4(1),p4y(1)],[p4(2),p4y(2)],[p4(3),p4y(3)],'Color','green','LineWidth',2)
 line([p4(1),p4z(1)],[p4(2),p4z(2)],[p4(3),p4z(3)],'Color','blue','LineWidth',2)
 line([p5(1),p5x(1)],[p5(2),p5x(2)],[p5(3),p5x(3)],'Color','red','LineWidth',2)
 line([p5(1),p5y(1)],[p5(2),p5y(2)],[p5(3),p5y(3)],'Color','green','LineWidth',2)
 line([p5(1),p5z(1)],[p5(2),p5z(2)],[p5(3),p5z(3)],'Color','blue','LineWidth',2)
 line([p6(1),p6x(1)],[p6(2),p6x(2)],[p6(3),p6x(3)],'Color','red','LineWidth',2)
 line([p6(1),p6y(1)],[p6(2),p6y(2)],[p6(3),p6y(3)],'Color','green','LineWidth',2)
 line([p6(1),p6z(1)],[p6(2),p6z(2)],[p6(3),p6z(3)],'Color','blue','LineWidth',2)

if show_val == 1

    patch(fv0,'FaceColor',       [0.3 0.3 0.5], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15);
         camlight('headlight');  


    fv1 = stl_T(T{1}*Trans(0,0,-0.222),fv1);
    patch(fv1,'FaceColor',       [0.3 0.3 0.5], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15);
         camlight('headlight');     

 
    fv2 = stl_T(T{1}*T{2}*Trans(0,0,0.205/2)*Rotz(pi),fv2);
    patch(fv2,'FaceColor',       [0.3 0.3 0.5], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15);
         camlight('headlight');     

 
    fv3 = stl_T(T{1}*T{2}*T{3}*Rotz(pi/2)*Trans(0,0,0.161/2),fv3);
    patch(fv3,'FaceColor',       [0.3 0.3 0.5], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15);
         camlight('headlight');    


    fv4 = stl_T(T{1}*T{2}*T{3}*T{4}*Trans(0,-0.005,-0.083),fv4);
    patch(fv4,'FaceColor',       [0.3 0.3 0.5], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15);
         camlight('headlight');         



    fv5 = stl_T(T{1}*T{2}*T{3}*T{4}*T{5}*Rotz(-pi/2)*Trans(0,0,-0.069),fv5);
    patch(fv5,'FaceColor',       [0.3 0.3 0.5], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.15);
         camlight('headlight');       

    fv6 = stl_T(T{1}*T{2}*T{3}*T{4}*T{5}*T{6}*Trans(0,0,-0.06),fv6);
    patch(fv6,'FaceColor',       [1.0 0.0 1.0], ...
             'EdgeColor',       'none',        ...
             'FaceLighting',    'gouraud',     ...
             'AmbientStrength', 0.3);
         camlight('headlight');            
    % Fix the axes scaling, and set a nice view angle
    axis('image');
    view([-135 35]);
end
drawnow;
hold off;
end