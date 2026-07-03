%% Step response to a half-sine pulse input using lsim
clearvars, clc, close all

%% System
G = tf(146.25,[1 3 146.25]);

%% Time vector and input
T  = 0.5;                  % pulse parameter (s)
dt = 0.001;                % time step
t  = 0:dt:2.5;             % t in [0, 2.5] s

r = zeros(size(t));
idx = (t >= 0) & (t <= T/2);
r(idx) = sin(2*pi*t(idx)/T);   % half-sine pulse, zero after T/2

%% Simulate with lsim (system initially at rest)
y = lsim(G,r,t);

%% Plot
figure
plot(t,r,'b--','LineWidth',1.2,'DisplayName','r(t) input')
hold on
plot(t,y,'r','LineWidth',1.5,'DisplayName','y(t) output')
hold off
xlabel('time (s)')
ylabel('amplitude')
title('Response of G(s) = 146.25 / (s^2 + 3s + 146.25) to half-sine pulse')
grid on
legend(Location="best")
