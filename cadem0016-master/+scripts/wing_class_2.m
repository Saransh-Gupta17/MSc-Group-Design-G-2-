clear; clc; close all

%% INPUT PARAMETERS

span_baseline = 64.8;
span_folding  = 70.3;

MTOM = 374000;
g = 9.81;
n = 2.5;

wing_area = 599.8;

% Assume mean aerodynamic chord approximation
c_mean = wing_area / span_baseline;

% Distance between CP and shear centre
d_cp_sc = 0.1 * c_mean;

N = 200;

%% TOTAL LOAD

W = MTOM * g;
L_total = n * W;

%% BASELINE

[y_b,q_b,V_b,M_b,T_b,Mroot_baseline] = wing_beam(span_baseline,L_total,N,d_cp_sc);

%% FOLDING

[y_f,q_f,V_f,M_f,T_f,Mroot_folding] = wing_beam(span_folding,L_total,N,d_cp_sc);

%% PRINT RESULTS

fprintf('Root bending moment baseline: %.2f MNm\n',Mroot_baseline/1e6)
fprintf('Root bending moment folding : %.2f MNm\n',Mroot_folding/1e6)

fprintf('Increase due to folding: %.1f %%\n', ...
((Mroot_folding-Mroot_baseline)/Mroot_baseline)*100)

%% LIFT DISTRIBUTION PLOT

figure
plot(y_b,q_b/1000,'LineWidth',2)
title('Lift Distribution (Baseline)')
xlabel('Spanwise Location (m)')
ylabel('Lift per unit span (kN/m)')
grid on

%% TORQUE DISTRIBUTION

figure
plot(y_b,T_b/1000,'LineWidth',2)
title('Torque Distribution (Baseline)')
xlabel('Spanwise Location (m)')
ylabel('Torque (kNm)')
grid on

%% BENDING MOMENT PLOTS

figure
plot(y_b,M_b/1e6,'LineWidth',2)
title('Baseline Bending Moment')
xlabel('Spanwise Location (m)')
ylabel('Moment (MNm)')
grid on

figure
plot(y_f,M_f/1e6,'LineWidth',2)
title('Folding Bending Moment')
xlabel('Spanwise Location (m)')
ylabel('Moment (MNm)')
grid on

%% ===============================
% FUNCTION MUST BE AT END
%% ===============================

function [y,q,V,M,T,M_root] = wing_beam(span,L_total,N,d_cp_sc)

b = span;

% span stations
y = linspace(0,b/2,N);

% elliptical lift distribution
q0 = (4*L_total)/(pi*b);
q = q0 * sqrt(1 - (2*y/b).^2);

% Shear force
V = cumtrapz(flip(y), flip(q));
V = flip(V);

% Bending moment
M = cumtrapz(flip(y), flip(V));
M = flip(M);

% Torque distribution
% T = Lift per unit span * moment arm
T = q * d_cp_sc;

% Root bending moment
M_root = M(1);

end