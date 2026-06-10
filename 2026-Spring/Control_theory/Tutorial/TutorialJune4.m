clearvars, clc;
syms x1 x2 x3
RHS = [-(3*x1+x3)/(x2+1);
       -(x2+2*x3)/(x2+3);
       x1];

xdot = RHS
x = [x1; x2; x3];
J = jacobian(RHS, x)

A = subs(J, {x1, x2, x3}, {0, 0, 0})

eigenvalues = eig(A)

ev = double(eigenvalues);
real_parts = real(ev)

if all(real_parts < 0)
    disp('所有特征值实部 < 0，系统在平衡点处局部渐近稳定')
else
    disp('存在实部 >= 0 的特征值，系统不稳定或无法判断')
end


