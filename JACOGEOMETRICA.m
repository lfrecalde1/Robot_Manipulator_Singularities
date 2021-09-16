
%condiciones iniciales Robot 
L1=295; L2=230; L3=50; L4=270; L6=70; L5=0;
q1(1)=0; 
q2(1)=-pi/2;
q3(1)=0;
q4(1)=0;
q5(1)=0;
q6(1)=0;

syms th d a alpha 
A= [ cos(th) -cos(alpha)*sin(th) sin(alpha)*sin(th) a*cos(th),
     sin(th) cos(alpha)*cos(th) -sin(alpha)*cos(th) a*sin(th),
     0       sin(alpha)          cos(alpha)            d     ,
     0        0               0                1  ];
%% Link1
syms th1 L1 
A1=subs(A,{th, d, a, alpha},{th1,L1,0,-pi/2});

%% Link2
syms th2 L2 
A2=subs(A,{th, d, a, alpha},{th2, 0, L2, 0});
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
T1=simplify(A1);
T2=simplify(A1*A2);
T3=simplify(A1*A2*A3);
T4=simplify(A1*A2*A3*A4);
T5=simplify(A1*A2*A3*A4*A5);
T6=simplify(A1*A2*A3*A4*A5*A6);

At2=subs(T6,{L1,L2,L3,L4,L6},{295,230,50,270,70});
At3=double(subs(subs(At2,{th1,th2,th3,th4,th5,th6},{0,-pi/3,pi/6,pi/2,0,0})));
%Metodo geometrico NUMERICO
Z0=[0; 0;1]
Z1=T1(1:3,3);
Z2=T2(1:3,3);
Z3=T3(1:3,3);
Z4=T4(1:3,3);
Z5=T5(1:3,3);
Z6=T6(1:3,3);

P1=T6(1:3,4)-[0; 0;0];
P2=T6(1:3,4)-T1(1:3,4);
P3=T6(1:3,4)-T2(1:3,4);
P4=T6(1:3,4)-T3(1:3,4);
P5=T6(1:3,4)-T4(1:3,4);
P6=T6(1:3,4)-T5(1:3,4);
Z0xP1=simplify(cross(Z0,P1));
Z1xP2=simplify(cross(Z1,P2));
Z2xP3=simplify(cross(Z2,P3));
Z3xP4=simplify(cross(Z3,P4));
Z4xP5=simplify(cross(Z4,P5));
Z5xP6=simplify(cross(Z5,P6));
J1=[Z0xP1;Z0];
J2=[Z1xP2;Z1];
J3=[Z2xP3;Z2];
J4=[Z3xP4;Z3];
J5=[Z4xP5;Z4];
J6=[Z5xP6;Z5];
Jg=[J1, J2, J3, J4, J5, J6]
J1g=subs(Jg,{L1,L2,L3,L4,L6},{295,230,50,270,70});
J2g=double(subs(J1g,{th1,th2,th3,th4,th5,th6},{0,-pi/3,pi/6,pi/2,0,0}))
% 
% %Metodo geometrico parciales
J11 = simplify(jacobian(T6(1:3,4),th1));
J22 = simplify(jacobian(T6(1:3,4),th2));
J33 = simplify(jacobian(T6(1:3,4),th3));
J44 = simplify(jacobian(T6(1:3,4),th4));
J55 = simplify(jacobian(T6(1:3,4),th5));
J66 = simplify(jacobian(T6(1:3,4),th6));
Jgp = [J11 J22 J33 J44 J55 J66;Z0 Z1 Z2 Z3 Z4 Z5];
% J1gp=subs(Jgp,{L1,L2,L3,L4,L6},{295,230,50,270,70});
% J2gp=double(subs(J1gp,{th1,th2,th3,th4,th5,th6},{0,-pi/2,-pi/2,0,0,0}));
