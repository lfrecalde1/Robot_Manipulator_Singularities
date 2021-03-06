function [f] = manipulador_optimo(z,Q,R,hd,h,q,l,ts,N,k,vreal,x,obj,bandera,W)

%a) Acciones de control
vc = z ; %z=[u,w,q1_p,q2_p,q3_p,q4_p]';
VC=[z(1,:)';z(2,:)';z(3,:)';z(4,:)';z(5,:)';z(6,:)'];

th1=q(1);
th2=q(2);
th3=q(3);
th4=q(4);
th5=q(5);
th6=q(6);



%generacion del vector vacio para los errores de control
L= zeros(N*3,1);

L(1:3,1)=Fun(hd(:,1),h(:,1));

%% Generacion parea los secundarios
q1= [th1; th2; th3; th4; th5; th6];

J= Jacobiano(q1, l);

B = zeros(N,1);

B(1,1)= singular(J);



%% Variable auxiliar para poder almacenar el vector total de los errores del sistema
control_size = size(h);

%% auxiliar
aux1=control_size(1)+1;

for i=1:N-1

    v= vc(:,i);
    q1= [th1(i); th2(i); th3(i); th4(i); th5(i); th6(i)];
    J= Jacobiano(q1, l);
   
    h(:,i+1)=ts*J*v+h(:,i);
    
    th1(i+1) = th1(i)+v(1)*ts;
    th2(i+1) = th2(i)+v(2)*ts;
    th3(i+1) = th3(i)+v(3)*ts;
    th4(i+1) = th4(i)+v(4)*ts;
    th5(i+1) = th5(i)+v(5)*ts;
    th6(i+1) = th6(i)+v(6)*ts;
    
    %% Generacion del vector de los errores del sistema
    L(aux1:aux1+2,1)=(Fun(hd(:,k+i),h(:,i+1)));
    
    J1 =Jacobiano([th1(i+1); th2(i+1); th3(i+1); th4(i+1); th5(i+1); th6(i+1)], l);
    B(i+1,1) = singular(J1);
    
    aux1=aux1+control_size(1);
    

end
%% seccion para Seleccionar si se quiere o no evasion de obstaculos
Q = Q*eye(length(L));
R = R*eye(length(VC));
%W=  W*eye(length(B));
%% Funcion costo a minizar tomando en cuanta las acciones de control del sistema
f=L'*Q*L+VC'*R*VC-W*sum(B);

end
function [F] = Fun(hd,h)
% Generacion de los os errores del sistema
he = hd-h;
F=he;
end

function [F] = singular(J)
% Generacion de los os errores del sistema
F = det(J*J');
end