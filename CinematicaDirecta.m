function [x ,y, z] = CinematicaDirecta(q ,L)
th1=q(1);
th2=q(2);
th3=q(3);
th4=q(4);
th5=q(5);
th6=q(6);
L1=L(1);
L2=L(2);
L3=L(3);
L4=L(4);
L5=L(5);
L6=L(6);
T11=sin(th6)*(cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + cos(th6)*(cos(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - sin(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)));
T12=cos(th6)*(cos(th4)*sin(th1) + sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - sin(th6)*(cos(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - sin(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)));
T13=- sin(th5)*(sin(th1)*sin(th4) - cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) - cos(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2));
T14=L2*cos(th1)*cos(th2) - L4*sin(th2 + th3)*cos(th1) - L6*sin(th2 + th3)*cos(th1)*cos(th5) + L3*cos(th1)*cos(th2)*cos(th3) - L3*cos(th1)*sin(th2)*sin(th3) - L6*sin(th1)*sin(th4)*sin(th5) + L6*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5) - L6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5);
T21=- sin(th6)*(cos(th1)*cos(th4) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) - cos(th6)*(cos(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + sin(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)));
T22=sin(th6)*(cos(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + sin(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))) - cos(th6)*(cos(th1)*cos(th4) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)));
T23=sin(th5)*(cos(th1)*sin(th4) + cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) - cos(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2));
T24=L2*cos(th2)*sin(th1) - L4*sin(th2 + th3)*sin(th1) - L6*sin(th2 + th3)*cos(th5)*sin(th1) + L3*cos(th2)*cos(th3)*sin(th1) + L6*cos(th1)*sin(th4)*sin(th5) - L3*sin(th1)*sin(th2)*sin(th3) - L6*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5) + L6*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5);
T31=sin(th2 + th3)*sin(th4)*sin(th6) - cos(th6)*(cos(th2 + th3)*sin(th5) + sin(th2 + th3)*cos(th4)*cos(th5));
T32=sin(th6)*(cos(th2 + th3)*sin(th5) + sin(th2 + th3)*cos(th4)*cos(th5)) + sin(th2 + th3)*cos(th6)*sin(th4);
T33=sin(th2 + th3)*cos(th4)*sin(th5) - cos(th2 + th3)*cos(th5);
T34=L1 - L4*cos(th2 + th3) - L3*sin(th2 + th3) - L2*sin(th2) + (L6*sin(th2 + th3)*sin(th4 + th5))/2 - L6*cos(th2 + th3)*cos(th5) - (L6*sin(th4 - th5)*sin(th2 + th3))/2;

R=[T11 T12 T13;
   T21 T22 T23;
   T31 T32 T33];
%% Matriz ZYX
Yaw=atan2(R(3,2),R(3,3));
Pitch=atan2(-R(3,1),sqrt(R(3,2)^2+ R(3,3)^2));
Roll=atan2(R(2,1),R(1,1));
ZYX=[Yaw,Pitch,Roll];
P=[T14; T24; T34];
x=P(1);
y=P(2);
z=P(3);
rotm=R;
Rota=R;
eul= rotm2eul(Rota);
end

