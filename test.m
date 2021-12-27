addpath("mr")
clear
close all;
L = 1
J = 6
H = 500;
t_step = 0.1;
method =5;

w = [0,0,1;
    0,0,1;
    0,0,1;
    0,0,1;
    0,0,1;
    0,0,1;
    0,0,1]
p = [0,0,0;
     L,0,0;
     2*L,0,0;
     3*L,0,0;
     4*L,0,0;
     5*L,0,0;
     6*L,0,0]
Slist = w_p_to_Slist(w,p,J);
M = [1,0,0,7*L;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1]
q0 =[0,0,0,0,0,0]';
qT = -[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2]';
q_traj = JointTrajectory(q0,qT,H*t_step,H+1,method);
v_traj = JointVelTrajectory(q0,qT,H*t_step,H+1,method);

T = FKinSpace(M,Slist,q0);
Gpick = FKinSpace(M,Slist,q0);
Gplace = FKinSpace(M,Slist,qT);


x_H=[]
for i = 1:1:H+1
    qq = q_traj(i,:);
    x_H=[x_H,qq];
end
for i = 1:1:H+1
    vv = v_traj(i,:);
    x_H=[x_H,vv];
end
x_H = x_H';

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
amax=[100,100,100,100,100,100]';
b_up=zeros(J*(H+1),1);
b_down=repmat(amax,H+1,1);
b_down(end-J+1:end)=0;
b= [b_up;b_down];
lb = [b_up;-b_down];
ub = [b_up;b_down];

A_start_constraint = [eye(J),zeros(J,J*(H+1)*2-J)];
A_end_constraint = [zeros(J,J*(H+1)-J),eye(J),zeros(J,J*(H+1))];
b_start_constraint = x_H(1:J);
b_end_constraint = x_H((H)*J+1:(H+1)*J);

A = [A;A_start_constraint];
A = [A;A_end_constraint];

A_vel_start_constraint = [zeros(J,J*(H+1)),eye(J),zeros(J,J*(H+1)-J)];
b_vel_start_constraint = zeros(J,1)

A = [A;A_vel_start_constraint];
b= [b;b_vel_start_constraint]
lb= [lb;b_start_constraint];
lb= [lb;b_end_constraint];
lb=[lb;b_vel_start_constraint];
ub= [ub;b_start_constraint];
ub= [ub;b_end_constraint];
ub= [ub;b_vel_start_constraint];


prob = osqp;
q = zeros((H+1)*2*J,1);
prob.setup(P, q, A, lb, ub, 'alpha', 1);
res = prob.solve();
x_H_result=res.x;
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
figure(1);
plot(new_q_list(:,1));
hold on;
plot(q_traj(:,1),'r--');


figure(2);
plot(new_v_list(:,1));
hold on;
plot(v_traj(:,1),'r--')