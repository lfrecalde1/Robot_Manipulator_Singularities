function [J] = Jacobiano(q, L)

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

J11=L4*sin(th2 + th3)*sin(th1) - L2*cos(th2)*sin(th1) + L6*sin(th2 + th3)*cos(th5)*sin(th1) - L3*cos(th2)*cos(th3)*sin(th1) - L6*cos(th1)*sin(th4)*sin(th5) + L3*sin(th1)*sin(th2)*sin(th3) + L6*cos(th2)*cos(th3)*cos(th4)*sin(th1)*sin(th5) - L6*cos(th4)*sin(th1)*sin(th2)*sin(th3)*sin(th5);
J12=-cos(th1)*(L4*cos(th2 + th3) + L2*sin(th2) + L6*cos(th2 + th3)*cos(th5) + L3*cos(th2)*sin(th3) + L3*cos(th3)*sin(th2) - L6*cos(th2)*cos(th4)*sin(th3)*sin(th5) - L6*cos(th3)*cos(th4)*sin(th2)*sin(th5));
J13=-cos(th1)*(L4*cos(th2 + th3) + L6*cos(th2 + th3)*cos(th5) + L3*cos(th2)*sin(th3) + L3*cos(th3)*sin(th2) - L6*cos(th2)*cos(th4)*sin(th3)*sin(th5) - L6*cos(th3)*cos(th4)*sin(th2)*sin(th5));
J14=-L6*sin(th5)*(cos(th4)*sin(th1) - cos(th1)*cos(th2)*cos(th3)*sin(th4) + cos(th1)*sin(th2)*sin(th3)*sin(th4));
J15=L6*sin(th2 + th3)*cos(th1)*sin(th5) - L6*cos(th5)*sin(th1)*sin(th4) + L6*cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3) - L6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5);
J16=0;
J21=L2*cos(th1)*cos(th2) - L4*sin(th2 + th3)*cos(th1) - L6*sin(th2 + th3)*cos(th1)*cos(th5) + L3*cos(th1)*cos(th2)*cos(th3) - L3*cos(th1)*sin(th2)*sin(th3) - L6*sin(th1)*sin(th4)*sin(th5) + L6*cos(th1)*cos(th4)*sin(th2)*sin(th3)*sin(th5) - L6*cos(th1)*cos(th2)*cos(th3)*cos(th4)*sin(th5);
J22=-sin(th1)*(L4*cos(th2 + th3) + L2*sin(th2) + L6*cos(th2 + th3)*cos(th5) + L3*cos(th2)*sin(th3) + L3*cos(th3)*sin(th2) - L6*cos(th2)*cos(th4)*sin(th3)*sin(th5) - L6*cos(th3)*cos(th4)*sin(th2)*sin(th5));
J23=-sin(th1)*(L4*cos(th2 + th3) + L6*cos(th2 + th3)*cos(th5) + L3*cos(th2)*sin(th3) + L3*cos(th3)*sin(th2) - L6*cos(th2)*cos(th4)*sin(th3)*sin(th5) - L6*cos(th3)*cos(th4)*sin(th2)*sin(th5));
J24=L6*sin(th5)*(cos(th1)*cos(th4) + cos(th2)*cos(th3)*sin(th1)*sin(th4) - sin(th1)*sin(th2)*sin(th3)*sin(th4));
J25=L6*sin(th2 + th3)*sin(th1)*sin(th5) + L6*cos(th1)*cos(th5)*sin(th4) + L6*cos(th4)*cos(th5)*sin(th1)*sin(th2)*sin(th3) - L6*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th1);
J26=0;
J31=0;
J32=L4*sin(th2 + th3) - L3*cos(th2 + th3) - L2*cos(th2) + (L6*cos(th2 + th3)*sin(th4 + th5))/2 + L6*sin(th2 + th3)*cos(th5) - (L6*sin(th4 - th5)*cos(th2 + th3))/2;
J33=L4*sin(th2 + th3) - L3*cos(th2 + th3) + L6*sin(th2 + th3)*cos(th5) + L6*cos(th2 + th3)*cos(th4)*sin(th5);
J34=-L6*sin(th2 + th3)*sin(th4)*sin(th5);
J35=L6*cos(th2 + th3)*sin(th5) + L6*sin(th2 + th3)*cos(th4)*cos(th5);
J36=0;
J=[J11 J12 J13 J14 J15 J16,
   J21 J22 J23 J24 J25 J26,
   J31 J32 J33 J34 J35 J36];

end

