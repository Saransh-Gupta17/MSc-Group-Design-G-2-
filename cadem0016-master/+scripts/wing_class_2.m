clear; clc; close all

%% ==================================
% Aircraft Inputs
%% ==================================

span = 70.3;          % wingspan (m)
S    = 489;           % wing area (m^2)
MTOM = 374000;        % kg

g = 9.81;

%% ==================================
% Load cases
%% ==================================

n_cases = [2.5 2.0 2.3]; % manoeuvre, landing, gust

W = MTOM*g;
L_cases = n_cases * W;

%% ==================================
% Span discretisation
%% ==================================

N = 400;
y = linspace(0,span/2,N);

%% ==================================
% Bending moments for each case
%% ==================================

M_roots = zeros(size(L_cases));

for i = 1:length(L_cases)

    q0 = (4*L_cases(i))/(pi*span);
    q  = q0*sqrt(1-(2*y/span).^2);

    V = cumtrapz(flip(y),flip(q));
    V = flip(V);

    M = cumtrapz(flip(y),flip(V));
    M = flip(M);

    M_roots(i) = M(1);

end

M_design = max(M_roots);

fprintf("Design bending moment: %.2f MNm\n",M_design/1e6)

%% ==================================
% Wing geometry
%% ==================================

lambda = 0.3;                    % taper ratio
c_root = 2*S/(span*(1+lambda));  % root chord

box_height = 0.12*c_root;        % wing box height
flange_width = 0.4*c_root;       % effective bending flange

fprintf("Root chord: %.2f m\n",c_root)
fprintf("Wing box height: %.2f m\n",box_height)

%% ==================================
% Material
%% ==================================

sigma_allow = 250e6;   % Pa
rho_al = 2800;         % kg/m3

%% ==================================
% Section modulus requirement
%% ==================================

Z_req = M_design / sigma_allow;

%% ==================================
% Root skin thickness
%% ==================================

A_skin_root = (2*Z_req)/box_height;
t_root = A_skin_root/(2*flange_width);

fprintf("Root skin thickness: %.1f mm\n",t_root*1000)

%% ==================================
% Spanwise thickness distribution
%% ==================================

t_span = t_root * sqrt(1-(2*y/span).^2);

%% ==================================
% Skin volume (integrated)
%% ==================================

skin_volume = 2 * trapz(y, flange_width .* t_span);

%% ==================================
% Spar volume (tapering)
%% ==================================

t_spar_root = 0.02;    % 20 mm
t_spar_tip  = 0.005;   % 5 mm

t_spar_span = linspace(t_spar_root,t_spar_tip,N);

spar_volume = 2 * trapz(y, box_height .* t_spar_span);

%% ==================================
% Rib estimation
%% ==================================

rib_spacing = 0.6;
n_ribs = span / rib_spacing;

rib_area = flange_width * box_height;
rib_volume = n_ribs * rib_area * 0.0015;  % light truss ribs

%% ==================================
% Structural volume
%% ==================================

structure_volume = skin_volume + spar_volume + rib_volume;

structure_mass = rho_al * structure_volume;

%% ==================================
% Control surfaces
%% ==================================

control_surface_mass = 0.05 * structure_mass;

wing_mass = structure_mass + control_surface_mass;

fprintf("Wing structural mass: %.1f tonnes\n",wing_mass/1000)

%% ==================================
% Plot load cases
%% ==================================

figure
bar(M_roots/1e6)
xticklabels(["Manoeuvre","Landing","Gust"])
ylabel("Root Bending Moment (MNm)")
title("Wing Load Cases")
grid on

%% ==================================
% Plot thickness distribution
%% ==================================

figure
plot(y,t_span*1000,'LineWidth',2)
xlabel("Spanwise location (m)")
ylabel("Skin thickness (mm)")
title("Spanwise Skin Thickness Distribution")
grid on