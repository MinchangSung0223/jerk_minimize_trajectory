clear

H = 9
J = 6
% t_step = 0.01;
syms t_step
syms zobstacle
q = sym('q',[H+1,J],'real')
amax = sym('amax',[J,1],'real')
v = sym('v',[H+1,J],'real')
Jk = sym('Jk',[6,J],'real');
FK = sym('FK',[6,1],'real');



x_H=[]

for i = 1:1:H+1
    qq = q(i,:);
    x_H=[x_H,qq];
end

for i = 1:1:H+1
    vv = v(i,:);
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
Adl=zeros((H+1)*J,(H+1)*J)
Adr=-eye((H+1)*J)/t_step;
for i =1:1:H+1
    for j =1:1:H+1
        if((i-j)==-1)
             Adr((i-1)*J+1:i*J,(j-1)*J+1:j*J)=eye(J)/t_step;
        end
    end
end

A = [[Aul,Aur];[Adl,Adr]];
A((H)*J+1:(H)*J+J,:) = [];
A(end-J+1:end,:) = [];

b_up=zeros(J*(H),1);
b_up = sym('b_up',[J*(H),1],'real');
b_up(1:end)=0;

b_down=repmat(amax,H+1,1);
b_down(end-J+1:end)=[];
b = [b_up;b_down]
lb= [b_up;-b_down];
ub = [b_up;b_down];


% A*x_H+b

R0 = [1 0 0 0 0 0;0 1 0 0 0 0 ; 0 0 1 0 0 0];
q0 = x_H(1:J);
A_start_constraint = R0*[repmat(zeros(J,J),1,0),Jk,repmat(zeros(J,J),1,2*(H+1)-(1))];
qend = x_H(end-J+1:end);
A_end_constraint = R0*[repmat(zeros(J,J),1,H+1-1),Jk,repmat(zeros(J,J),1,2*(H+1)-(H+1))];

A_vel_start_constraint = [zeros(J,J*(H+1)),eye(J),zeros(J,J*(H+1)-J)];
A_vel_end_constraint = [zeros(J,J*(H+1)),zeros(J,J*(H+1)-J),eye(J)];
A_q_min_max_constraint = [zeros((H+1)*J,(H+1)*J),eye((H+1)*J)]
A_v_min_max_constraint = [zeros((H+1)*J,(H+1)*J),eye((H+1)*J)]

A_obstacle_constraint = []
getZ = [0 0 0 0 0 1];
for i=2:1:H
    qi = x_H((i-1)*J+1:i*J);
    A_obstacle = getZ*[repmat(zeros(J,J),1,i-1),Jk,repmat(zeros(J,J),1,2*(H+1)-(i))];
    A_obstacle_constraint=[A_obstacle_constraint;A_obstacle];
end


b_start_constraint = x_H(1:J);
b_end_constraint = x_H((H)*J+1:(H+1)*J);
b_vel_start_constraint = zeros(J,1);
b_vel_end_constraint =  zeros(J,1);
b_q_min_constraint = repmat([-1,-1,-1,-1,-1,-1]',H+1,1);
b_q_max_constraint = repmat([1,1,1,1,1,1]',H+1,1);
b_obstacle_constraint = []
for i=2:1:H
    qi = x_H((i-1)*J+1:i*J);
    b_obstacle = zobstacle-FK(end)+getZ*Jk*qi;
    b_obstacle_constraint=[b_obstacle_constraint;b_obstacle];
end


A = [A;A_start_constraint;A_end_constraint; A_vel_start_constraint;A_vel_end_constraint;A_q_min_max_constraint;A_obstacle_constraint];
lb= [lb;b_start_constraint; b_end_constraint;b_vel_start_constraint;b_vel_end_constraint;b_q_min_constraint;b_obstacle_constraint];
ub= [ub;b_start_constraint; b_end_constraint;b_vel_start_constraint;b_vel_end_constraint;b_q_max_constraint;b_obstacle_constraint];


A*x_H-lb

% A=subs(A, t_step, 0.001)


