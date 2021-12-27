addpath('mr')
addpath('function')
addpath('mesh')

clear
close all;
%% Parameter Setup

L = 1
J = 6
H =30;
ITER = 100;
t_step_start = 0.1;
qmin = [-3.0543,-3.0543,-3.0543,-3.0543,-3.0543,-pi]';
qmax = [3.0543,3.0543,3.0543,3.0543,3.0543,pi]';
vmin = [-2.6180,-2.6180,-2.6180,-2.6180,-2.6180,-2.6180]';
vmax = [2.6180,2.6180,2.6180,2.6180,2.6180,2.6180]';
amax=[40,40,40,40,40,40]';
yconstraint_min = 0.4;
yconstraint_max = inf;

ax = -258
ay =7
light_z = 50000
hold_on = false;
xobastacle = 0.05
t_step=0.1
target_z = 0.1
zobstacle=target_z+0.2
    x=input("PRESS ENTER")
    close all;
    

    eps = [0.02;0.02;0.02;0.0;0.0;0.0];
    method =5;
    eomg = 0.01;
    ev = 0.001;
    
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
    Slist = [S1,S2,S3,S4,S5,S6];
    M= [1 0 0 0;...
        0 1 0 -W1-W2 ;...
        0 0 1 H1+H2+H3+H4;...
        0 0 0 1 ];
    
    M06 = [1 0 0 0;...
        0 0 -1 -W1-W2 ;...
        0 1 0 H1+H2+H3;...
        0 0 0 1 ];
    
    M05= [1 0 0 0;...
        0 1 0 -W1 ;...
        0 0 1 H1+H2+H3;...
        0 0 0 1 ];
    
    
    q0 = [   -0.2149   -0.4691   -1.0256   -0.0000   -1.6469   -0.2149]';
    Gpick = [-1   0    0    0.45;
              0   1    0   -0.2;
              0   0    -1    target_z;
             0         0         0    1.0000];
   Rpick = eye(4);
    Rpick(1:3,1:3) =eul2rotm([0.0,0.0,0]);
   
    Gmid = [-1   0    0    0.5;
              0   1    0   0.0;
              0   0    -1    0.7;
             0         0         0    1.0000];     
    Gplace = [-1   0    0    0.45;
              0   1    0   0.2;
              0   0    -1    target_z;
             0         0         0    1.0000]; 

    Gplace = Gplace*Rpick;
    
    [q0, success0] = IKinSpace(Slist, M, Gpick, q0, eomg, ev)
    [qT, successT] = IKinSpace(Slist, M, Gplace, q0, eomg, ev)
    
    q_traj = JointTrajectory(q0,qT,H*t_step,H+1,method);
    v_traj = JointVelTrajectory(q0,qT,H*t_step,H+1,method);
    
    q_traj_temp = []
    v_traj_temp=[]
    for i = 1:1:H+1
        qq = q_traj(i,:);
        vv = v_traj(i,:);
        q_traj_temp=[q_traj_temp,qq];
        v_traj_temp=[v_traj_temp,vv];
    end
    x_H=[q_traj_temp';v_traj_temp'];
    
    %% Set P
    P = zeros((H+1)*J*2,(H+1)*J*2);
    Pv = eye((H+1)*J)*2;
    for i =1:1:(H+1)
            for j =1:1:(H+1)
                if(abs(i-j)==1)
                    Pv((i-1)*J+1:(i)*J,(j-1)*J+1:(j)*J)=-1*eye(J);
                end
            end
    end
    P((H+1)*J+1:end,(H+1)*J+1:end)=Pv;
    
    
    %% Set A
    Aur= -t_step*eye((H+1)*J);
    Aul = -eye((H+1)*J);
    for i =1:1:H+1
    
        for j =1:1:H+1
    
            if((i-j)==-1)
    
                 Aul((i-1)*J+1:i*J,(j-1)*J+1:j*J)=eye(J);
    
            end
    
        end
    
    end
    Adl=zeros((H+1)*J,(H+1)*J);
    Adr=-eye((H+1)*J)/t_step;
    for i =1:1:H+1
    
        for j =1:1:H+1
    
            if((i-j)==-1)
    
                 Adr((i-1)*J+1:i*J,(j-1)*J+1:j*J)=eye(J)/t_step;
    
            end
    
        end
    
    end
    A = [[Aul,Aur];[Adl,Adr]];
    A((H)*J+1:(H)*J+J,:) = zeros(J,2*(H+1)*J);
    b_up=zeros(J*(H+1),1);
    b_down=repmat(amax,H+1,1);
    b_down(end-J+1:end)=0;
    b= [b_up;b_down];
    b_q_min_constraint = repmat(qmin,H+1,1);
    b_q_max_constraint = repmat(qmax,H+1,1);
    b_v_min_constraint = repmat(vmin,H-1,1);
    b_v_min_constraint =[[0 0 0 0 0 0]' ;b_v_min_constraint;[0 0 0 0 0 0]';];
    b_v_max_constraint = repmat(vmax,H-1,1);
    b_v_max_constraint =[[0 0 0 0 0 0]' ;b_v_max_constraint;[0 0 0 0 0 0]';];
    
    
    lb = [b_up;-b_down];
    ub = [b_up;b_down];
    
    Jk = eye(J)
    
    
    A_vel_start_constraint = [zeros(J,J*(H+1)),eye(J),zeros(J,J*(H+1)-J)];
    A_vel_end_constraint = [zeros(J,J*(H+1)),zeros(J,J*(H+1)-J),eye(J)];
    A_q_min_max_constraint = [eye((H+1)*J),zeros((H+1)*J,(H+1)*J)];
    A_v_min_max_constraint = [zeros((H+1)*J,(H+1)*J),eye((H+1)*J)];
    
    A_obstacle_constraint = [];
    A_obstacle_constraint_X = [];

    getR = [1 0 0 0 0 0];
    getP = [1 0 0 0 0 0];
    getY = [1 0 0 0 0 0];
    getZ = [0 0 0 0 0 1];
    getX = [0 0 0 1 0 0];
    for  i=1+2:1:H+1-2
        qi = x_H((i-1)*J+1:i*J);
        Jk = JacobianSpace(Slist,qi);
        A_obstacle = getZ*[repmat(zeros(J,J),1,i-1),Jk,repmat(zeros(J,J),1,2*(H+1)-(i))];
        A_obstacle_X = getX*[repmat(zeros(J,J),1,i-1),Jk,repmat(zeros(J,J),1,2*(H+1)-(i))];
        A_obstacle_constraint=[A_obstacle_constraint;A_obstacle];
        A_obstacle_constraint_X= [A_obstacle_constraint_X;A_obstacle_X];
    end
    R0 =eye(6);
    q0 = x_H(1:J);
    qend = x_H((H)*J+1:(H+1)*J);
    
    A_start_constraint = R0*[repmat(zeros(J,J),1,0),JacobianSpace(Slist,q0),repmat(zeros(J,J),1,2*(H+1)-(1))];    
    A_end_constraint = R0*[repmat(zeros(J,J),1,H+1-1),JacobianSpace(Slist,qend),repmat(zeros(J,J),1,2*(H+1)-(H+1))];
    
    
    
    
    
    
    
    
%     %b_start_constraint = R0*(se3ToVec(MatrixLog6(TransInv(FKinSpace(M,Slist,q0))*Gpick))+JacobianSpace(Slist,q0)*q0);
%     %b_end_constraint = R0*(se3ToVec(MatrixLog6(TransInv(FKinSpace(M,Slist,qend))*Gplace))+JacobianSpace(Slist,qend)*qend);%x_H((H)*J+1:(H+1)*J);
      b_start_constraint = R0*(se3ToVec(MatrixLog6(TransInv(FKinSpace(M,Slist,q0))*Gpick))+JacobianSpace(Slist,q0)*q0);
      b_end_constraint = R0*(se3ToVec(MatrixLog6(TransInv(FKinSpace(M,Slist,qend))*Gplace))+JacobianSpace(Slist,qend)*qend);%x_H((H)*J+1:(H+1)*J);

 %   b_start_constraint = R0*(se3ToVec(Gpick)-se3ToVec(FKinSpace(M,Slist,q0))+JacobianSpace(Slist,q0)*q0);%x_H(1:J);
 %   b_end_constraint = R0*(se3ToVec(Gplace)-se3ToVec(FKinSpace(M,Slist,qend))+JacobianSpace(Slist,qend)*qend);%x_H((H)*J+1:(H+1)*J);

    b_vel_start_constraint = zeros(J,1);
    b_vel_end_constraint =  zeros(J,1);
    b_obstacle_constraint = []
    b_obastalce_constraint_z_lb=[]
    b_obastalce_constraint_z_ub = []  
    b_obastalce_constraint_x_lb=[]
    b_obastalce_constraint_x_ub=[]
        
    for i=1+2:1:H+1-2
        qi = x_H((i-1)*J+1:i*J);
        FK = se3ToVec(FKinSpace(M,Slist,qi))
        Jk = JacobianSpace(Slist,qi);
        b_obstacle = zobstacle-FK(end)+getZ*Jk*qi;
        b_obstacle_ub = inf;
        b_obstacle_constraint=[b_obstacle_constraint;b_obstacle];
        b_obastalce_constraint_z_lb=[b_obastalce_constraint_z_lb;zobstacle+0.05-FK(end)+getZ*Jk*qi;];
        b_obastalce_constraint_z_ub=[b_obastalce_constraint_z_ub;zobstacle+0.15-FK(end)+getZ*Jk*qi;];        
        %b_obastalce_constraint_z_ub=[b_obastalce_constraint_z_ub;inf]; 
        b_obastalce_constraint_x_lb=[b_obastalce_constraint_x_lb;yconstraint_min-FK(end-2)+getX*Jk*qi;];
        b_obastalce_constraint_x_ub=[b_obastalce_constraint_x_ub;yconstraint_max-FK(end-2)+getX*Jk*qi;];
    end
    
    
    A_ieq = A;
    lb_ieq = lb;
    ub_ieq = ub;
    
    A = [A_ieq;A_start_constraint;A_end_constraint; ;A_obstacle_constraint;];
    lb= [lb_ieq;b_start_constraint-eps; b_end_constraint-eps;;b_obastalce_constraint_z_lb;];
    ub= [ub_ieq;b_start_constraint+eps; b_end_constraint+eps;;b_obastalce_constraint_z_ub;];
   
    prob = osqp;
    q = zeros((H+1)*2*J,1);
    prob.setup(P, q, A, lb, ub, 'alpha', 1);
    res = prob.solve();
    prev_x_H = x_H;
    x_H=res.x;
    
    for k  = 1:1:ITER
        q0 = x_H(1:J);
        qend = x_H((H)*J+1:(H+1)*J);
        
        A_start_constraint = R0*[repmat(zeros(J,J),1,0),JacobianSpace(Slist,q0),repmat(zeros(J,J),1,2*(H+1)-(1))];    
        A_end_constraint = R0*[repmat(zeros(J,J),1,H+1-1),JacobianSpace(Slist,qend),repmat(zeros(J,J),1,2*(H+1)-(H+1))];
        b_start_constraint = R0*(se3ToVec(MatrixLog6(TransInv(FKinSpace(M,Slist,q0))*Gpick))+JacobianSpace(Slist,q0)*q0);
        b_end_constraint = R0*(se3ToVec(MatrixLog6(TransInv(FKinSpace(M,Slist,qend))*Gplace))+JacobianSpace(Slist,qend)*qend);%x_H((H)*J+1:(H+1)*J);

    
        A_obstacle_constraint = [];
        for  i=1+2:1:H+1-2
            qi = prev_x_H((i-1)*J+1:i*J);
            A_obstacle = getZ*[repmat(zeros(J,J),1,i-1),Jk,repmat(zeros(J,J),1,2*(H+1)-(i))];
            A_obstacle_X = getX*[repmat(zeros(J,J),1,i-1),Jk,repmat(zeros(J,J),1,2*(H+1)-(i))];
            A_obstacle_constraint=[A_obstacle_constraint;A_obstacle];
            A_obstacle_constraint_X= [A_obstacle_constraint_X;A_obstacle_X];
        end
        b_obastalce_constraint_z_lb=[]
        b_obastalce_constraint_z_ub = []  
        for i=1+2:1:H+1-2
                qi = prev_x_H((i-1)*J+1:i*J);
                FK = se3ToVec(FKinSpace(M,Slist,qi))
                b_obastalce_constraint_z_lb=[b_obastalce_constraint_z_lb;zobstacle+0.05-FK(end)+getZ*Jk*qi;];
                b_obastalce_constraint_z_ub=[b_obastalce_constraint_z_ub;zobstacle+0.15-FK(end)+getZ*Jk*qi;];        
                %=[b_obastalce_constraint_z_ub;inf]; 
        end

        A = [A_ieq;A_start_constraint;A_end_constraint;A_obstacle_constraint;];
        lb= [lb_ieq;b_start_constraint-eps; b_end_constraint-eps;b_obastalce_constraint_z_lb];
        ub= [ub_ieq;b_start_constraint+eps; b_end_constraint+eps;b_obastalce_constraint_z_ub];

        prob.update('Ax',A,'l', lb, 'u', ub);
        prev_x_H=x_H;
        x_H=res.x

    end
    
    x_H_result=x_H;
    %% VISUALIZE
    
    new_q_list=[];
    for i =1:1:H+1
        new_q = x_H_result(i*J+1:(i+1)*J);
        new_q_list=[new_q_list;new_q'];
    end
    new_q_list(end,:) = new_q_list(end-1,:) ;
    new_v_list=[]
    for i =H+1:1:2*(H+1)-1
        new_v = x_H_result(i*J+1:(i+1)*J);
        new_v_list=[new_v_list;new_v'];
    end
    figure(2);
    hold on;
    for j = 1:1:J
        subplot(J,1,j)
        plot(new_q_list(:,1));
        hold on;
        plot(q_traj(:,1),'r--');
        hold off;
    end
    hold off;
    figure(3);
    hold on;
    for j = 1:1:J
        subplot(J,1,j)
        plot(new_v_list(:,1));
        hold on;
        plot(v_traj(:,1),'r--')
        hold off;
        
    end
    hold off;
   
    FKlist = []

      for i = 1:1:length(new_q_list)
    
          thetalist = new_q_list(i,:)';
          FK_ = se3ToVec(FKinSpace(M,Slist,thetalist));
          FKlist=[FKlist;FK_'];
          drawrobot_screw(thetalist,1,ax,ay,light_z,xobastacle,zobstacle,hold_on,t_step);
      end
      figure(4)
      subplot(6,1,1)
      plot(FKlist(:,1))

      subplot(6,1,2)
    plot(FKlist(:,2))
      subplot(6,1,3)
        plot(FKlist(:,3))
      subplot(6,1,4)
      plot(FKlist(:,end-2))
      title("x")
      subplot(6,1,5)
      plot(FKlist(:,end-1))
      title("y")
      subplot(6,1,6)
      plot(FKlist(:,end))
      hold on;
      plot(zobstacle*ones(size(FKlist(:,end))),'r:')
      title("z")
