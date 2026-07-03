%% Franklin Example 7.2 p. 484
clearvars, clc, close all

m = 1500;
b = 60;
u = 750;

A = [0 1; 0 -b/m]
B = [0; 1/m]

% Consider part (b), which implies a different definition of the C matrix than part (a).
C = [0 1]
D = 0

sys = ss(A,B,C,D)

%% To compare: example 2.1 (p. 48)
G = tf(1/m,[1 b/m])

s  = tf('s');
G1 = (1/m)/(s + b/m)

%% Calculation of step response
[ys,ts] = step(sys);
[yg,tg] = step(G);

%% Also do simulation in a SIMULINK model and pull SIMULINK data in here for comparison.
simdat = sim('Franklin_ex_7_2_sim.slx');

%% --- Alternative to SIMULINK: direct numerical integration with ode45 ---
% Instead of building a block diagram in Simulink, we can simulate the
% same state-space system x' = A*x + B*u, y = C*x + D*u directly by
% numerically integrating the ODE with ode45. This reproduces what the
% Simulink model does (state-space block + step input) without needing
% a .slx file at all.

tspan = [0 max(ts)];      % simulate over the same time span as step(sys)
x0    = [0; 0];           % zero initial conditions

odefun = @(t,x) A*x + B*u;   % u is the step input amplitude (applied for t>=0)

[t_ode, x_ode] = ode45(odefun, tspan, x0);
y_ode = (C*x_ode')' + D*u;   % output: y = C*x + D*u

%% Plot: state-space, transfer function, SIMULINK, and ode45 comparison
figure(1)
plot(ts,u*ys,'y','LineWidth',2,'DisplayName','State-space')
hold on
plot(tg,u*yg,'r-.','DisplayName','Transfer Function')
% SIMULINK data:
plot(simdat.ScopeData.time,simdat.ScopeData.signals.values,...
    'b.','DisplayName','SIMULINK')
% ode45 data (no Simulink needed):
plot(t_ode,y_ode,'ko','MarkerSize',4,'DisplayName','ode45 (no Simulink)')
hold off
xlabel('time (s)')
ylabel('velocity (m/s)')
grid on
legend(Location="southeast")
