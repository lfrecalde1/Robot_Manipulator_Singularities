%%Propuesta control de posicion x y z 
clc;
clear all;
close all;
ts=0.1; 
t=0:ts:10;

%%Condiciones iniciales Robot
L1=295; L2=230; L3=50; L4=270; L6=70; L5=0;
L=[L1; L2; L3; L4; L5; L6]*(1/1000);
q1(1)=0; q2(1)=-pi/2; q3(1)=-pi/6; q4(1)=0; q5(1)=0; q6(1)=0;%posicion inicial del robot
q= [q1(1); q2(1); q3(1); q4(1); q5(1); q6(1)];
%Cinematica directa
[xr(1), yr(1), zr(1)]= CinematicaDirecta(q ,L);

%Refencias deseadas 
%POSICION DESEADA cambiar aqui 
xrd=0.5*ones(1,length(t));%columnas 1xts
yrd=0.3*ones(1,length(t));
zrd=0.575*ones(1,length(t));


xrdp=0*ones(1,length(t));% trayectoria derivada en funcion tiempo
yrdp=0*ones(1,length(t));
zrdp=0*ones(1,length(t));

%% Generacion de la matrix de estados deseados
Hd = [xrd;yrd;zrd];

% MATRIZ DE GANANCIA
K1= 1*eye(3,3); %EN POSICION 
K2= 2*eye(3,3);%suaviza las acciones  ley de control
%PONDERACIONES PESO PARA EL ESPACIO NULO
K3=[1,0,0,0,0,0 ;...
    0,10,0,0,0,0;...
    0,0,10,0,0,0;...
    0,0,0,1,0,0;...
    0,0,0,0,1,0;...
    0,0,0,0,0,1];
    
%Genereacion de configuracion deseada espacio nulo 
q1d=0*ones(1,length(t));
q2d=-pi/2*ones(1,length(t));
q3d=pi/4*ones(1,length(t));
q4d=0*ones(1,length(t));
q5d=0*ones(1,length(t));
q6d=0*ones(1,length(t));

%% Definicion del Controlador basado en Optiomizacion
%% Este valor cambiar si desea cambiar el horizonte de predciccion si se aumenta se demora mas optimizando
N=5;

%% RESTRICCION PARA LAS ACCIONES DE CONTROL
lb = [-2.5,-2.5,-2.5,-2.5,-2.5,-2.5]';
ub = [ 2.5, 2.5, 2.5, 2.5, 2.5, 2.5]';
LB=[];
UB =[];

for index=1:N-1
    LB=[LB;lb];
    UB=[UB;ub];
end

%% CONFIGURACION DEL METODO DE OPTIMIZACION A UTILIZAR
solver='fmincon';
b='Display';
c='off';
d='Algorithm';
e='sqp';
options=optimoptions(solver,b,c,d,e);

%% generacion de la solucion iniciale del sistema
z0=[0*ones(1,N-1);...
    0*ones(1,N-1);...
    0*ones(1,N-1);...
    0*ones(1,N-1);...
    0*ones(1,N-1);...
    0*ones(1,N-1)];

%% Variables auxiliares para el controlador
%% Generacion de los objetos a evadir
Obj=0;

%% Valores para matriz de ganancia
Q=1; %% Posicion 
R=0.01; %% Si se aumenta esa las acciones de control son mas suaves para puede que no se llega a erroes de cero
W = 10; %% determiante
%% bandera para objetos
bandera=0;  %% 0 sin objetos 1 con objetos

%% 
v_real = 0;

x= 0;
for k=1:length(t)-N
    %% Generacion del tic
    tic
    %errores de control
    xre(k)=xrd(k)-xr(k);
    yre(k)=yrd(k)-yr(k);
    zre(k)=zrd(k)-zr(k);
    %generación de vector de estados de trabajo
    h=[xr(k);yr(k);zr(k)];
    hd=[xrd(k);yrd(k);zrd(k)];%rpy
    he=hd-h;%puntos
    hdp=[xrdp(k);yrdp(k);zrdp(k)];%rpyp es 0 xq hay punto mouse
   
    %generación de vector de estados de trabajo de joints
    q= [q1(k); q2(k); q3(k); q4(k); q5(k); q6(k)];
    qd=[q1d(k); q2d(k); q3d(k); q4d(k); q5d(k); q6d(k)];%null
    qe=qd-q;%q0p /null
  
    J= Jacobiano(q, L);
    determinante(k) = det(J*J');
    %Seudoinversa de %6x3
    Jseudo=J'*inv(J*J');
    Nulo=eye(6,6)-Jseudo*J;
    %LEY DE CONTROL
    %qp_ref=Jseudo*(hdp+K2*tanh((K2)^(-1)*K1*he))+Nulo*K3*qe;% EN EL INSTANTE DEL TIEMPO puntos normal hdp es 0 xq no hay trayectoria
    [qp_ref] = MPC(z0,Q,R,Hd,h,q,L,ts,N,k,v_real,x,Obj,bandera,W,LB,UB,options);

    q1p_ref(k)=qp_ref(1,1);
    q2p_ref(k)=qp_ref(2,1);
    q3p_ref(k)=qp_ref(3,1);
    q4p_ref(k)=qp_ref(4,1);
    q5p_ref(k)=qp_ref(5,1);
    q6p_ref(k)=qp_ref(6,1);
    %Integral

    q1(k+1)=q1(k)+q1p_ref(k)*ts;
    q2(k+1)=q2(k)+q2p_ref(k)*ts;
    q3(k+1)=q3(k)+q3p_ref(k)*ts;
    q4(k+1)=q4(k)+q4p_ref(k)*ts;
    q5(k+1)=q5(k)+q5p_ref(k)*ts;
    q6(k+1)=q6(k)+q6p_ref(k)*ts;

    %llamar a cinematica directa para actualizar posicion del robot
    qact=[q1(k+1); q2(k+1); q3(k+1); q4(k+1); q5(k+1); q6(k+1)];
    
    [xr(k+1), yr(k+1), zr(k+1)]= CinematicaDirecta(qact ,L);%qact a partir de q+1
    
    z0 = [qp_ref(1,:);...
          qp_ref(2,:);...
          qp_ref(3,:);...
          qp_ref(4,:);...
          qp_ref(5,:);...
          qp_ref(6,:)];
      % sample time
   sample(k)=toc;

end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1,1:length(xre)),xre,'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(xre)),yre,'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1,1:length(xre)),zre,'Color',[26,115,160]/255,'linewidth',1);hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolucion de errores de control}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1,1:length(xre)),q1p_ref,'Color',[223,67,85]/255,'linewidth',1); hold on
plot(t(1,1:length(xre)),q2p_ref,'Color',[56,171,217]/255,'linewidth',1); hold on
plot(t(1,1:length(xre)),q3p_ref,'Color',[32,185,29]/255,'linewidth',1); hold on
plot(t(1,1:length(xre)),q4p_ref,'Color',[217,204,30]/255,'linewidth',1); hold on
plot(t(1,1:length(xre)),q5p_ref,'Color',[83,57,217]/255,'linewidth',1); grid on
plot(t(1,1:length(xre)),q6p_ref,'Color',[23,87,217]/255,'linewidth',1); grid on
grid on;
legend({'$\dot{q}{1{ref_c}}$','$\dot{q}{2{ref_c}}$','$\dot{q}{3{ref_c}}$','$\dot{q}{4{ref_c}}$','$\dot{q}{5{ref_c}}$','$\dot{q}{6{ref_c}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Valores velocidad}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
subplot(2,1,2)
plot(t(1,1:length(q1)),q1,'Color',[223,67,85]/255,'linewidth',1); hold on
plot(t(1,1:length(q1)),q2,'Color',[56,171,217]/255,'linewidth',1); hold on
plot(t(1,1:length(q1)),q3,'Color',[32,185,29]/255,'linewidth',1); hold on
plot(t(1,1:length(q1)),q4,'Color',[217,204,30]/255,'linewidth',1); hold on
plot(t(1,1:length(q1)),q5,'Color',[83,57,217]/255,'linewidth',1); grid on
plot(t(1,1:length(q1)),q6,'Color',[23,87,217]/255,'linewidth',1); grid on
grid on;
legend({'$q_{1_{ref_c}}$','$q_{2_{ref_c}}$','$q_{3_{ref_c}}$','$q_{4_{ref_c}}$','$q_{5_{ref_c}}$','$q_{6_{ref_c}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Valores posicion}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1,1:length(xre)),determinante,'Color',[226,76,44]/255,'linewidth',1); hold on;

grid on;
legend({'Det'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolucion del determinante}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot(t(1,1:length(xre)),sample,'Color',[226,76,44]/255,'linewidth',1); hold on;

grid on;
legend({'Sample'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Sample time}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
