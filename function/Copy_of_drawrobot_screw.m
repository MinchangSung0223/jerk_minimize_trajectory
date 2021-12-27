function drawrobot_screw(thetalist,show_val,fv1,fv2,fv3,fv4,fv5,fv6)


thetalist
 
H1 = 0.3;
H2 = 0.45;
H3 = 0.350;
H4 = 0.228;
W1 = 0.0035;
W2 = 0.183;
S1 = [0; 0;  1;  0; 0;  0];
S2 =        [0; -1;  0; H1;0 ;  0];
S3 =        [0; -1;  0; H1+H2; 0; 0];
S4 =        [0; 0;   1; -W1; 0; 0];
S5 =        [0; -1;  0; H1+H2+H3; 0; 0];
S6 =        [0; 0;  1; -W1-W2;0; 0];
matS1 = MatrixExp6(VecTose3(S1)*thetalist(1));
matS2 = MatrixExp6(VecTose3(S2)*thetalist(2));
matS3 = MatrixExp6(VecTose3(S3)*thetalist(3));
matS4 = MatrixExp6(VecTose3(S4)*thetalist(4));
matS5 = MatrixExp6(VecTose3(S5)*thetalist(5));
matS6 = MatrixExp6(VecTose3(S6)*thetalist(6));


 p0 = [0,0,0,1]';
 p0x = [0.1,0,0,1]';
 p0y = [0,0.1,0,1]';
 p0z = [0,0,0.1,1]';
for k=1:1:6
T{k} = eye(4); 
end
M = [1 0 0 0;...
    0 1 0 -W1-W2 ;...
    0 0 1 H1+H2+H3+H4;...
    0 0 0 1 ];
Slist = [S1,S2,S3,S4,S5,S6];
T07 = matS1*matS2*matS3*matS4*matS5*matS6*M;
T{6} = T07;


M = [1 0 0 0;...
    0 0 -1 -W1-W2 ;...
    0 1 0 H1+H2+H3;...
    0 0 0 1 ];
Slist = [S1,S2,S3,S4,S5];
T06 =  matS1*matS2*matS3*matS4*matS5*M;
T{5} = T06;

M = [1 0 0 0;...
    0 1 0 -W1 ;...
    0 0 1 H1+H2+H3;...
    0 0 0 1 ];
Slist = [S1,S2,S3,S4];
T05 = matS1*matS2*matS3*matS4*M;
T{4} = T05;


M = [-1 0 0 0;...
    0 0 -1 0;...
    0 -1 0 H1+H2;
    0 0 0 1 ];
Slist = [S1,S2,S3];
T04 =matS1*matS2*matS3*M;
T{3} = T04;

M = [0  -1 0 0;...
    0   0 -1 0;...
    1   0  0 H1;
    0   0  0 1 ];
Slist = [S1,S2];
T03 = matS1*matS2*M;

T{2} = T03;
M = [1 0 0 0;...
    0 1 0 0;...
    0 0 1 H1;
    0 0 0 1 ];
Slist = [S1];
T02 =matS1*M;
T{1} = T02;


 p1 = T{1}*p0;
 p1x = T{1}*p0x;
 p1y = T{1}*p0y;
 p1z = T{1}*p0z;
 
 p2 = T{2}*p0;
 p2x = T{2}*p0x;
 p2y =T{2}*p0y;
 p2z =T{2}*p0z;
 
 p3 =T{3}*p0;
 p3x =T{3}*p0x;
 p3y = T{3}*p0y;
 p3z =T{3}*p0z;
 
 p4 = T{4}*p0;
 p4x =T{4}*p0x;
 p4y = T{4}*p0y;
 p4z =T{4}*p0z;
 
 p5 = T{5}*p0;
 p5x =T{5}*p0x;
 p5y =T{5}*p0y;
 p5z =T{5}*p0z;
 
 p6 = T{6}*p0;
 p6x =T{6}*p0x;
 p6y =T{6}*p0y;
 p6z =T{6}*p0z;
 

 
 plot3([0,2 0 0 0 -2 0 0 0 0 0 0 ],[0,0 0 2 0 0 0 -2 0 0 0 0 ],[0,0 0 0 0 0 0 0 0 0 0 2])
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
drawnow;
hold off;
end