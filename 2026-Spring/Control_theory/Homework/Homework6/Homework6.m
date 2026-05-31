A = readmatrix("HW6_A.xlsx");
B = [2; 0; 1.5; 0];
C = [1 0 2 0];
D = 0;

AA = [A B; C 0] ;
bb = [0; 0; 0; 0; 1] ;
n = size(A,1);   % state dimension = 4

%% --CHECK CONTROLLABILITY & OBSERVABILITY--
Co = ctrb(A, B);
Ob = obsv(A, C);
 
fprintf('=== Controllability & Observability ===\n');
fprintf('Rank of controllability matrix: %d (should be %d)\n', rank(Co), n);
fprintf('Rank of observability matrix:   %d (should be %d)\n\n', rank(Ob), n);
 
if rank(Co) < n || rank(Ob) < n
    error('System is not fully controllable or observable. Check A, B, C.');
end

%% --- (a) Regulator and Estimator Design ---
% Open-loop poles
ol_poles = eig(A);
fprintf('Open-loop poles:\n'); disp(ol_poles);

% Regulator poles (well into LHP, good damping)
reg_poles = [-4+4i, -4-4i, -10, -12];
K = place(A, B, reg_poles);
fprintf('State feedback gain K:\n'); disp(K);
fprintf('Closed-loop (regulator) poles:\n'); disp(eig(A - B*K));

% Observer poles (~3x faster than regulator)
obs_poles = [-20+8i; -20-8i; -25+5i; -25-5i];
L = place(A', C', obs_poles)';
fprintf('Observer gain L:\n'); disp(L);
fprintf('Observer (A-LC) poles:\n'); disp(eig(A - L*C));

%% --(b) tracking system --
sys_cl = A - B*K;
Nbar = -inv(C * (sys_cl \ B));   % scalar feedforward gain
fprintf('Feedforward gain Nbar = %.4f\n', Nbar);

Acl_b = [A,       -B*K;
         L*C,   A-L*C-B*K];

Bcl_b = [B*Nbar;  
         B*Nbar];  

Ccl_b = [C, zeros(1,n)];
Dcl_b = 0;

sys_b = ss(Acl_b, Bcl_b, Ccl_b, Dcl_b);
fprintf('Part (b) closed-loop poles:\n'); disp(eig(Acl_b));

% 2nd methods 
xx = AA \ bb; 

Nx = xx(1:4);
Nu = xx(5);
N_bar = Nu+ K * Nx;
display(Nx);
display(Nu);
display(Nbar);


%% --(c) Internal Model Control for robust tracking --
A_ext = [0,  C ;      
         zeros(n,1), A];  
B_ext = [0; B];           
C_ext = [0, C];           
fprintf('Extended system controllability rank: %d (should be %d)\n', rank(ctrb(A_ext, B_ext)), n+1);
 
% Regulator poles for extended system (5 poles needed)
reg_poles_ext = [-5+5i, -5-5i, -10, -15, -20];
K_ext = place(A_ext, B_ext, reg_poles_ext);
 
K1 = K_ext(1);        % 积分器增益 (标量)
K0 = K_ext(2:end);    % 状态反馈增益 (1×4)

fprintf('K1 (integrator gain) = %.4f\n', K1);
fprintf('K0 (state gains) = [%s]\n', num2str(K0));
 
% Observer for original plant (same as part a)
% u = -K_x*x_hat - K_i*x_i, with x_hat from observer
 
% Full augmented state for simulation: z = [x; x_hat; x_i]  (9 states)
A_aug_c = [ 0,      C,              zeros(1,n)  ;
           -B*K1,   A,              -B*K0       ;
           -B*K1,   L*C,    (A - L*C - B*K0)   ];

B_aug_c = [-1; zeros(n,1); zeros(n,1)];  
C_aug_c = [0, C, zeros(1,n)];           

sys_c = ss(A_aug_c, B_aug_c, C_aug_c, 0);

display(sys_c) ;

%% --(d) unit step resposes of the tracking system --
t = linspace(0, 5, 1000);
 
[y_b, t_b] = step(sys_b, t);
[y_c, t_c] = step(sys_c, t);
 
figure('Name','Part (d): Step Responses - Nominal Plant','NumberTitle','off');
plot(t_b, y_b, 'b-',  'LineWidth', 2); hold on;
plot(t_c, y_c, 'r--', 'LineWidth', 2);
yline(1, 'k:', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y(t)', 'FontSize', 12);
title('Unit Step Responses - Nominal Plant', 'FontSize', 14);
legend('(b) Autonomous Estimator (Franklin §7.9.1)', ...
       '(c) Internal Model Control (Integral Action)', ...
       'Reference r = 1', ...
       'Location', 'southeast', 'FontSize', 11);
xlim([0 5]); ylim([-0.2 1.5]);
 
% Steady-state values
fprintf('Steady-state y_b(5s) = %.4f\n', y_b(end));
fprintf('Steady-state y_c(5s) = %.4f\n', y_c(end));

%% --(e) Robustness: aged plant (A scaled by 1.5), controllers unchanged --
aging_factor = 1.50;          % 50 % increase in A
A_aged = aging_factor * A;

% --- (b) with aged plant: estimator/regulator/Nbar all use original A ---
Acl_b_aged = [A_aged,   -B*K;
              L*C,      A - L*C - B*K];
Bcl_b_aged = [B*Nbar;
              B*Nbar];
Ccl_b_aged = [C, zeros(1,n)];
sys_b_aged = ss(Acl_b_aged, Bcl_b_aged, Ccl_b_aged, 0);

fprintf('\n=== (e) Aged-plant closed-loop poles ===\n');
fprintf('Part (b) aged poles:\n'); disp(eig(Acl_b_aged));

% --- (c) with aged plant: integrator + observer (designed for original A) ---
A_aug_c_aged = [ 0,        C,            zeros(1,n)  ;
                -B*K1,     A_aged,       -B*K0       ;
                -B*K1,     L*C,          A - L*C - B*K0 ];
B_aug_c_aged = [-1; zeros(n,1); zeros(n,1)];
C_aug_c_aged = [0, C, zeros(1,n)];
sys_c_aged = ss(A_aug_c_aged, B_aug_c_aged, C_aug_c_aged, 0);

fprintf('Part (c) aged poles:\n'); disp(eig(A_aug_c_aged));

% Stability check
if any(real(eig(Acl_b_aged)) >= 0) || any(real(eig(A_aug_c_aged)) >= 0)
    warning('One of the aged closed-loop systems is unstable. Try a smaller aging factor.');
end

% Simulate
[y_b_aged, t_b_aged] = step(sys_b_aged, t);
[y_c_aged, t_c_aged] = step(sys_c_aged, t);

figure('Name','Part (e): Step Responses - Aged Plant (A x 1.5)','NumberTitle','off');
plot(t_b_aged, y_b_aged, 'b-',  'LineWidth', 2); hold on;
plot(t_c_aged, y_c_aged, 'r--', 'LineWidth', 2);
yline(1, 'k:', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Output y(t)', 'FontSize', 12);
title(sprintf('Unit Step Responses - Aged Plant (A scaled by %.2f)', aging_factor), 'FontSize', 14);
legend('(b) Autonomous Estimator', ...
       '(c) Internal Model Control', ...
       'Reference r = 1', ...
       'Location', 'southeast', 'FontSize', 11);
xlim([0 5]); ylim([-0.2 1.5]);

fprintf('Aged steady-state y_b(5s) = %.4f  (error = %.4f)\n', y_b_aged(end), 1 - y_b_aged(end));
fprintf('Aged steady-state y_c(5s) = %.4f  (error = %.4f)\n', y_c_aged(end), 1 - y_c_aged(end));

