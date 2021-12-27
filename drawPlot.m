function [q1_list,q2_list]=drawPlot(x_H,x_H2,H,J)
    q1_list = [];
    q2_list = [];
    v1_list = [];
    v2_list = [];
    a1_list = [];
    a2_list = [];
    j1_list = [];
    j2_list = [];
    
    for i = 0:1:H
        q1 = x_H(i*J+1:(i+1)*J);
        q2 = x_H2(i*J+1:(i+1)*J);
        q1_list=[q1_list;q1'];
        q2_list=[q2_list;q2'];        
    end
    
    for i = (H+1):1:(H+1)*2-1
        v1 = x_H(i*J+1:(i+1)*J);
        v2 = x_H2(i*J+1:(i+1)*J);
        v1_list=[v1_list;v1'];
        v2_list=[v2_list;v2'];        
    end
    
    for i = (H+1)*2:1:(H+1)*3-1
        a1 = x_H(i*J+1:(i+1)*J);
        a2 = x_H2(i*J+1:(i+1)*J);
        a1_list=[a1_list;a1'];
        a2_list=[a2_list;a2'];        
    end
    
    for i = (H+1)*3:1:(H+1)*4-1
        j1 = x_H(i*J+1:(i+1)*J);
        j2 = x_H2(i*J+1:(i+1)*J);
        j1_list=[j1_list;j1'];
        j2_list=[j2_list;j2'];        
    end
    figure(2)
    title("Position")
    for j = 1:1:J
        subplot(J,1,j)
        plot(q1_list(:,j));
        hold on;
        plot(q2_list(:,j),'r--');
        hold off;
    end
    
     figure(3)
    title("Velocity")
     for j = 1:1:J
        subplot(J,1,j)
        plot(v1_list(:,j));
        hold on;
        plot(v2_list(:,j),'r--');
        hold off;
    end
    
        figure(4)
    title("Acceleration")
      for j = 1:1:J
        subplot(J,1,j)
        plot(a1_list(:,j));
        hold on;
        plot(a2_list(:,j),'r--');
        hold off;
    end
    
    figure(5)
    title("Jerk")
        
    for j = 1:1:J
        subplot(J,1,j)
        plot(j1_list(:,j));
        hold on;
        plot(j2_list(:,j),'r--');
        hold off;
    end
    
end