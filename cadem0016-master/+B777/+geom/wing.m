function [GeomObj, massObj] = wing(obj)
% STABILIZED CLASS II.5 WING STRUCTURAL SIZING
% 
% Same interface as original:
%   INPUT:  obj = ADP object with MTOM, Span, WingArea, Mstar, TLAR.M_c, WingPos
%   OUTPUT: [GeomObj, massObj] = geometry and mass objects
%
% KEY FIX: Spar thickness is now physics-based (from shear), not chord-dependent.
%          This removes MTOM^2 feedback and stabilizes MDO convergence.
%
% Stabilization achieved by:
%   1. Spar web thickness: t_spar ∝ Shear / Box_Height, NOT chord
%   2. Result: Mass ∝ MTOM (linear), not MTOM^2 (quadratic)
%   3. Feedback gain < 1 → convergent MDO

%% ========== MATERIAL & STRUCTURAL PARAMETERS ==========
g = 9.81;

% Material
rho = 2800;           % kg/m^3, aluminum
sigma_bending = 250e6;  % Pa, bending allowable
sigma_shear = 150e6;    % Pa, shear allowable (web stress)

% Knockdown factors
eta_struct = 0.5;     % Structural efficiency (bending)
eta_shear = 0.6;      % Shear allowable reduction

% Load cases (symmetric, design points)
n_cases = [2.5, 2.0, 2.3];

%% ========== EXTRACT INPUTS ==========
MTOM = obj.MTOM;
span = obj.Span;
WingArea = obj.WingArea;

%% ========== STEP 1: LOAD ANALYSIS & BENDING MOMENTS ==========
N = 400;  % spanwise discretization
y = linspace(0, span/2, N);

% Compute maximum root bending moment across load cases
M_roots = zeros(size(n_cases));

for i = 1:length(n_cases)
    % Design load factor
    n = n_cases(i);
    
    % Total lift per wing (half-wing assumption)
    L = n * MTOM * g / 2;
    
    % ===== ELLIPTICAL LIFT DISTRIBUTION =====
    % q(y) = q0 * sqrt(1 - (2y/span)^2)
    q0 = (4 * L) / (pi * span);
    q = q0 * sqrt(1 - (2*y/span).^2);
    
    % ===== SHEAR FORCE (numerical integration from tip to root) =====
    % V(y) = integral from y to span/2 of q(eta) deta
    % Flip-integrate to avoid accumulation errors at root
    V = flip(cumtrapz(flip(y), flip(q)));
    
    % ===== BENDING MOMENT (numerical integration) =====
    % M(y) = integral from y to span/2 of V(eta) deta
    M = flip(cumtrapz(flip(y), flip(V)));
    
    % Root bending moment (max)
    M_roots(i) = M(1);
end

% Design moment (worst case)
M_design = max(M_roots);

% Also keep shear distribution from design case (for spar sizing)
n_design = n_cases(1);
L_design = n_design * MTOM * g / 2;
q0_design = (4 * L_design) / (pi * span);
q_design = q0_design * sqrt(1 - (2*y/span).^2);
V_design = flip(cumtrapz(flip(y), flip(q_design)));

%% ========== STEP 2: PLANFORM GEOMETRY ==========
% Taper ratio based on Mach number
tr = -0.0083 * acosd(0.75 * obj.Mstar / obj.TLAR.M_c);

% Root chord
c_root = 2 * WingArea / (span * (1 + tr));

% Mean aerodynamic chord
c_mac = (2/3) * c_root * (1 + tr + tr^2) / (1 + tr);
obj.c_ac = c_mac;  % Store in ADP

% Box geometry (% of root chord)
box_h = 0.12 * c_root;      % Box depth
flange_w = 0.4 * c_root;    % Flange width (spar separation)

%% ========== STEP 3: SKIN SIZING (bending-driven) ==========
% Skin carries bending stress
% Z_required = M / (sigma_allow * eta_struct)
% Z = A * h for a rectangular section → A_skin = Z / h (skin area)
% t_skin = A_skin / width

Z_req = (M_design / sigma_bending) * eta_struct;
A_skin = (2 * Z_req) / box_h;  % Two flanges
t_root_skin = A_skin / (2 * flange_w);

% Cap skin thickness at 5% chord (manufacturing/buckling limit)
t_root_skin = min(t_root_skin, 0.05 * c_root);

% Skin taper (follows elliptical moment distribution)
% Moment reduces as sqrt(1 - (2y/span)^2) spanwise
chord_ratio = sqrt(1 - (2*y/span).^2);  % Normalized chord
t_skin_span = t_root_skin * chord_ratio;  % Skin thickness taper

% Skin volume (two surfaces)
skin_vol = 2 * trapz(y, flange_w .* t_skin_span);

%% ========== STEP 4: SPAR WEB SIZING (TIER 2 CORRECTION) ==========
% KEY FIX: Spar thickness based on SHEAR STRESS, not chord percentage.
%
% Web shear stress: tau = V / (t_web * h)
% Allowable: tau ≤ sigma_shear * eta_shear
% Therefore: t_web ≥ V / (sigma_shear * eta_shear * h)
%
% This makes t_web ∝ V (which ∝ MTOM), NOT t_web ∝ chord (which also ∝ MTOM).
% Result: Volume ∝ t_web * h * span ∝ MTOM, not MTOM^2 !

% Minimum gauge thickness (manufacturing constraint)
t_min_spar = 0.0015;  % 1.5 mm

% Maximum thickness (avoid over-sizing)
t_max_spar = 0.015;   % 15 mm

% Spar web thickness from shear at each station
t_spar_web = zeros(size(y));
for j = 1:N
    % Required thickness for shear
    t_required = V_design(j) / (sigma_shear * eta_shear * box_h);
    
    % Apply bounds
    t_spar_web(j) = max(t_required, t_min_spar);
    t_spar_web(j) = min(t_spar_web(j), t_max_spar);
end

% Spar volume (two webs, front and rear)
spar_vol = 2 * trapz(y, box_h .* t_spar_web);

%% ========== STEP 5: RIB SIZING ==========
% Ribs spaced at ~0.6 m, thickness 1.0 mm (standard)
rib_spacing = 0.6;  % m
n_ribs = (span / 2) / rib_spacing;
rib_thickness = 0.0010;  % m (1.0 mm)
rib_height = box_h;
rib_width = flange_w;

rib_vol = n_ribs * (rib_height * rib_width * rib_thickness);

%% ========== STEP 6: MASS CALCULATION ==========
% Total structural volume
total_vol = skin_vol + spar_vol + rib_vol;

% Structural mass
struct_mass = rho * total_vol;

% Add control surfaces (~5%)
wing_mass = struct_mass * 1.05;

%% ========== STEP 7: GEOMETRY OUTPUT ==========
% GeomObj not used in this model, return empty
GeomObj = [];

%% ========== STEP 8: MASS OUTPUT ==========
massObj = cast.MassObj( ...
    Name = "Wing_Structural", ...
    m    = wing_mass, ...
    X    = [obj.WingPos; 0]);

%% ========== DEBUG/LOGGING ==========
% Uncomment for diagnostics
% fprintf("Wing Sizing Summary:\n");
% fprintf("  MTOM: %.0f kg\n", MTOM);
% fprintf("  M_root: %.0f kN·m\n", M_design/1e6);
% fprintf("  Span: %.1f m\n", span);
% fprintf("  Root chord: %.2f m\n", c_root);
% fprintf("  Box height: %.3f m\n", box_h);
% fprintf("  t_skin_root: %.4f m (%.1f%% chord)\n", t_root_skin, 100*t_root_skin/c_root);
% fprintf("  t_spar_root: %.4f m (%.1f%% chord)\n", t_spar_web(1), 100*t_spar_web(1)/c_root);
% fprintf("  Skin volume: %.2f m^3\n", skin_vol);
% fprintf("  Spar volume: %.2f m^3\n", spar_vol);
% fprintf("  Rib volume: %.3f m^3\n", rib_vol);
% fprintf("  Total struct mass: %.0f kg\n", struct_mass);
% fprintf("  Wing mass (with ctrl): %.0f kg (%.1f%% MTOM)\n", ...
%     wing_mass, 100*wing_mass/MTOM);

end