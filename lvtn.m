clc; clear all; close all;
tsamp=0.05;
t=50;
samples=t/tsamp;

L1=0.141; L2=0.068; L3=0.096; L4=0.103;

L(1) = Link([0, L1, 0, pi/2, 0]);            % Link([theta,d,a,anfa])
L(2) = Link([0, L2, 0, pi/2, 0]);  
L(3) = Link([0, 0, L3, 0, 1])%, 0, [0 0 0],'qlim', [0 1]]);
L(4) = Link([0, L4, 0, 0, 0]);
L(3).qlim=[0 5];

rob = SerialLink(L,'name','Robot Tracking')    % SerialLink(L)

W = [-2 3.4 -2 2 -2 3];
mp=subplot(121)
hold on
grid on
view(3)
M = [0 pi/2 1 pi/2]
% a=rob.jacobe(M)
% a=rob.jacobn(M)
% a=rob.jacob0(M)
%rob.plot(M,);
rob.plot(M,'workspace',W)
T=rob.fkine(M)
% syms th1 th2 d3 th4
% q(th1, th2, d3, th4)=[th1 th2 d3 th4]
xx=0.3; yy=0.2;
lamda=diag([1 1 1 1 1 1]);
P=[3 xx yy+0.2; 3 xx -yy+0.2; 3 -xx -yy+0.2]';
plot3(P(1,:),P(2,:), P(3,:), 'k.');
%CAMERA
f=577.4948; fx=578.0391; fy=576.9504; cx=298.1673; cy=233.8321; s=0.1312 %In pixx=
cam=CentralCamera('focal',3*10e-6*f, 'pixel', [3 3]*10e-6, 'resolution', [640 480], 'pose', T)
cam.plot_camera

pd=cam.project(P)



%%
%change origin position


M=[pi/30 pi/2-pi/20 1.2 pi/2-pi/20]
subplot(121)
rob.plot(M,'workspace',W)
T=rob.fkine(M)
% syms th1 th2 d3 th4
% q(th1, th2, d3, th4)=[th1 th2 d3 th4]

lamda=diag([1 1 1 1 1 1]);
%CAMERA
cam.T=T
f=577.4948; fx=578.0391; fy=576.9504; cx=298.1673; cy=233.8321; s=0.1312 %In pixx=
%cam.k=[fx s cx; 0 fy cy; 0 0 1]
% cam.project(P, 'Tcam', T);
subplot(121)
cam.plot_camera

mp=subplot(122)
hold on
grid on
set(mp,'Ydir', 'reverse')
axis([0 640 0 480])
pbaspect([4 3 1])
title('Picture')
p=cam.project(P)
pd_plot=plot(pd(1,:), pd(2,:),'r');
p_plot=plot(p(1,:), p(2,:),'b');
plot([p(1,1) pd(1,1)], [p(2,1) pd(2,1)], 'm-.')
plot([p(1,2) pd(1,2)], [p(2,2) pd(2,2)], 'c-.')
plot([p(1,3) pd(1,3)], [p(2,3) pd(2,3)], 'g-.')
legend([pd_plot p_plot], {'pd', 'p'})
err=p-pd
err_a=[reshape(err,[],1)];
v_array=[0 0 0 0]';
e_xl=10;
t(1)=0
i=1;
vlim=[1.5 1.5 0.04 1.5] %rad/s & m/s
while ~isequal(abs(err)<e_xl, ones(2,3))
    i=i+1;
    t(i)=t(i-1)+tsamp;
    jR=rob.jacobe(M)
    jC=cam.visjac_p(p,1)
    v=pinv(jR)*pinv(jC)*(lamda*reshape(err,[],1))
    for index=1:4
        if v(index)<-vlim(index)
            v(index)=-vlim(index);
        elseif v(index)>vlim(index)
            v(index)=vlim(index);
        end
    end
    v_array=[v_array v]
    M=(M'+tsamp*v)'
    
    subplot(121)
    
    T=rob.fkine(M)
    cam.T=T.double*([diag([1 1 1]) [0 -0.2 0]'; [0 0 0 1]]);
    
%     p=cam.project(P, 'pose', T)
%     p=cam.project(P)%, 'pose', T)
    rob.plot(M,'workspace',W)
    cam.plot_camera
    subplot(122)
    hold off
    p=cam.project(P)
    pd_plot=plot(pd(1,:), pd(2,:),'r');
    hold on
    title('Picture')
    grid on
    set(mp,'Ydir', 'reverse')
    axis([0 640 0 480])
    pbaspect([4 3 1])
    p_plot=plot(p(1,:), p(2,:),'b');
    plot([p(1,1) pd(1,1)], [p(2,1) pd(2,1)], 'm-.')
    plot([p(1,2) pd(1,2)], [p(2,2) pd(2,2)], 'c-.')
    plot([p(1,3) pd(1,3)], [p(2,3) pd(2,3)], 'g-.')
    legend([pd_plot p_plot], {'pd', 'p'})
    err=pd-p
    err_a=[err_a reshape(err,[],1)];
end

figure(2)
title('Error and Velocity')
t=0:tsamp:(tsamp*(i-1));
subplot(131)
hold on
title('Error')
err_pl_str=["ex_1" "ey_1" "ex_2" "ey_2" "ex_3" "ey_3"]
for j=1:6
    plot(t,err_a(j,:),'LineWidth',2, 'DisplayName', err_pl_str(j));
end

grid on
xlabel('time (s)');
ylabel('Error (pixel)');
legend


subplot(132)
hold on
title('Revolute Joint Velocity');
v_pl=["v1" "v2" "v3" "v4"]
for j=[1 3 4]
    plot(t,v_array(j,:),'LineWidth',2, 'DisplayName', v_pl(j));
end

grid on
xlabel('time (s)');
ylabel('v (rad/s)');
legend

subplot(133)
hold on
title('Prismatic Joint Velocity');

plot(t,v_array(3,:),'LineWidth',2, 'DisplayName', v_pl(3));


grid on
xlabel('time (s)');
ylabel('v (m/s)');
legend
