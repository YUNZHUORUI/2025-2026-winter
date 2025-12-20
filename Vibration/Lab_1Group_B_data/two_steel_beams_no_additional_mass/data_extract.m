clear all, close all, clc

file1 = 'expdata.mat';
load(file1);


figure(1)
plot(t1_n,yg1)
grid on
xlabel('Time (s)')
ylabel('y acceleration (g)')
title('Free vibration 1DOF system')
hold off
