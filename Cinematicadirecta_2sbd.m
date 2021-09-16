%melfa mitsubishi 2sbd
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|        295|          0|    -1.5708|          0|
% |  2|    q2-pi/2|          0|        230|          0|          0|
% |  3|         q3|          0|         50|    -1.5708|          0|
% |  4|         q4|        270|          0|     1.5708|          0|
% |  5|         q5|          0|          0|    -1.5708|          0|
% |  6|         q6|         70|          0|          0|          0|
% +---+-----------+-----------+-----------+-----------+-----------+
L1=295; L2=230; L3=50; L4=270; L6=70; L5=0;
%%  Parameters 
syms th d a alpha 
A= [ cos(th) -cos(alpha)*sin(th) sin(alpha)*sin(th) a*cos(th),
     sin(th) cos(alpha)*cos(th) -sin(alpha)*cos(th) a*sin(th),
     0       sin(alpha)          cos(alpha)            d     ,
     0        0               0                1  ]
%% Link1
syms th1 L1 
A1=subs(A,{th, d, a, alpha},{th1,L1,0,-pi/2});

%% Link2
syms th2 L2 
A2=subs(A,{th, d, a, alpha},{th2 , 0, L2, 0});
%% Link3
syms th3 L3
A3=subs(A,{th, d, a, alpha},{th3, 0, L3, -pi/2});
%% Link2
syms th4  L4 
A4=subs(A,{th, d, a, alpha},{th4, L4, 0, pi/2});
%% Link5
syms th5   
A5=subs(A,{th, d, a, alpha},{th5, 0, 0, -pi/2});
%% Link
syms th6  L6 
A6=subs(A,{th, d, a, alpha},{th6, L6, 0, 0});
%% Matriz Homogenea
At=simplify(A1*A2*A3*A4*A5*A6);
At2=subs(At,{L1,L2,L3,L4,L6},{295,230,50,270,70});
At3=subs(At2,{th1,th2,th3,th4,th5,th6},{27.91*pi/180,18.87*pi/180,-pi/6,3.28*pi/180,10.32*pi/180,58.29*pi/180})
%% Matriz de posicion
 X=double(subs(At3(1,4)))
 Y=double(subs(At3(2,4)))
 Z=double(subs(At3(3,4)))
% %% Matriz ZYXAt
% Yat=atan2(At(3,2),At(3,3));
% Pat=atan2(-At(3,1),sqrt(At(1,1)^2+ At(2,1)^2));
% Rat=atan2(At(2,1),At(1,1));
% ZYXat=[Yat;Pat;Rat]
%% Matriz ZYX
Yaw=atan2(At3(3,2),At3(3,3));
Pitch=atan2(-At3(3,1),sqrt(At3(1,1)^2+ At3(2,1)^2));
Roll=atan2(At3(2,1),At3(1,1));
ZYX=[Yaw,Pitch,Roll]

%% Matriz XYZ
Ya=atan2(-At3(2,3),At3(3,3));%-
P=atan2(At3(1,3),sqrt(At3(1,1)^2+ At3(1,2)^2));
R=atan2(-At3(1,2),At3(1,1));
XYZPeter=[Ya,P,R]
%% Matriz ZYX
% Pitchy1=asin(At3(1,3))
% if Pitchy1>0 & Pitchy1<=0
% Pitchyy=atan2(-At3(3,1),sqrt(At3(3,2)^2+ At3(3,3)^2));
% Rollzz=atan2(At3(2,1)/cos(Pitchyy),At3(1,1)/cos(Pitchyy));
% Yawxx=atan2(At3(3,2)/cos(Pitchyy),At3(3,3)/cos(Pitchyy));
% ZYXtanp=[Yawxx,Pitchyy,Rollzz];
% 
% Pitchyy1=atan2(-At3(3,1),-sqrt(At3(3,2)^2+ At3(3,3)^2));
% Rollzz1=atan2(At3(2,1)/cos(Pitchyy1),At3(1,1)/cos(Pitchyy1));
% Yawxx1=atan2(At3(3,2)/cos(Pitchyy1),At3(3,3)/cos(Pitchyy1));
% ZYXtann=[Yawxx1,Pitchyy1,Rollzz1];

%% Matriz rpysin
% Rollz1=asin(At3(2,1)/cos(asin(-At3(3,1))));
% Pitchy1=asin(-At3(3,1));
% Yawx1=asin(At3(3,2)/cos(asin(-At3(3,1))));
% modelo=[Yawx1,Pitchy1,Rollz1];


rotm=At3(1:3,1:3);
Rota=double(subs(rotm))
eulZYX = rotm2eul(Rota)


 %% INVERSA
% R =[At3(1,1) At3(1,2) At3(1,3); At3(2,1) At3(2,2) At3(2,3); At3(3,1) At3(3,2) At3(3,3)];
% % A36=(A1*A2*A3)'*AR
% eul = rotm2eul(R)
% px=X;
% py=Y;
% pz=Z;
% % rx=Roll;
% % ry=Pitch;
% % rz=Yaw;
% rx= 2.35619;
% ry= 1.57079;
% rz= 0;
% ax = cos(rz) * cos(ry);
% ay = sin(rz) *cos(ry);
% az = -sin(ry);
%         
% p5x = px - (70) * ax;
% p5y = py - (70) * ay;
% p5z = pz - (70) * az;
% 
% q1= atan2(p5y, p5x);
% 
% C3 = (p5x^ 2 + p5y^2 + (p5z - 295)^2 - (230^ 2) - (50 + 230)^ 2) / (2 * 230 * (50 + 230));
% 
% q3 = atan2((1 -(C3^ 2))^0.5 , C3);
% 
% M = 270 + (50 + 230) * C3;
% N = (50 +230) * sin(q3);
% A = (p5x^2 + p5y^2)^ 0.5;
% B = p5z - 295;
% q2= atan2(M * A - N * B, N * A + M * B);
% C1 = cos(q1);
% C23 = cos(q2 + q3);
% S1 = sin(q1);
% S23 = sin(q2 + q3);
% bx = cos(rx) * sin(ry) *cos(rz) +sin(rx) * sin(rz);
% by = cos(rx) * sin(ry) *sin(rz) - sin(rx) * cos(rz);
% bz = cos(rx) * cos(ry);
% asx = C23 * (C1 * ax + S1 * ay) - S23 * az;
% asy = -S1 * ax + C1 * ay;
% asz = S23 * (C1 * ax + S1 * ay) + C23 * az;
% bsx = C23 * (C1 * bx + S1 * by) - S23 * bz;
% bsy = -S1 * bx + C1 * by;
% bsz = S23 * (C1 * bx + S1 * by) + C23 * bz;
% q4 = atan2(asy, asx);
% q5= atan2(cos(q4) * asx + sin(q4) * asy, asz);
% q6 = atan2(cos(q4) * bsy - sin(q4) * bsx, -bsz /sin(q5));
% INV=[q1,q2,q3,q4,q5,q6]
% 
% 
% 
% 
