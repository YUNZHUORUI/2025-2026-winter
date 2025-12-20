clearvars, clc

M = readmatrix("mass_matrixMATLAB.xlsx");
K = readmatrix("stiffness_matrixMATLAB.xlsx");
C = readmatrix("damping_matrixMATLAB.xlsx");

format long
disp('Upper left sub-half-matrix of K')
K(1:4,1:4)
disp('Lower right sub-half-matrix of K')
K(5:8,5:8)
disp('Upper right sub-half-matrix of K')
K(1:4,5:8)

shouldbezero8times8 = abs(K - transpose(K));
disp('shouldbezero8times8:')
for ii = 1:8
    fprintf(' %12.10f',shouldbezero8times8(ii,:))
    fprintf('\n')
end
fprintf('\n')

disp('Upper left sub-half-matrix of M')
M(1:4,1:4)
disp('Lower right sub-half-matrix of M')
M(5:8,5:8)
disp('Upper right sub-half-matrix of M')
M(1:4,5:8)

shouldbezero8times8 = abs(M - transpose(M));
disp('shouldbezero8times8:')
for ii = 1:8
    fprintf(' %12.10f',shouldbezero8times8(ii,:))
    fprintf('\n')
end
fprintf('\n')

disp('Upper left sub-half-matrix of C')
C(1:4,1:4)
disp('Lower right sub-half-matrix of C')
C(5:8,5:8)
disp('Upper right sub-half-matrix of C')
C(1:4,5:8)

shouldbezero8times8 = abs(C - transpose(C));
disp('shouldbezero8times8:')
for ii = 1:8
    fprintf(' %12.10f',shouldbezero8times8(ii,:))
    fprintf('\n')
end
fprintf('\n')
