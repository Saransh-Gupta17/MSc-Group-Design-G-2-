function [GeomObj, massObj] = fuselage(obj)
% Fuselage structural sizing and mass estimation for B777F
% Methods: Raymer Class I (MDO) and beam-based Class II.5 (FEDR)
% Load cases per CS-25 / Megson Ch.8 & Ch.10
% Author: Fuselage Structures Engineer

%% Fuselage Geometry
L_f = obj.L_total;

theta = linspace(0, pi, 101)';
Xs = [-sin(theta)*obj.CockpitLength, cos(theta)*obj.CabinRadius];

theta = linspace(pi, 0, 101)';
Xs = [Xs; ...
    sin(theta)*obj.CabinRadius*2*2.48 + (obj.CabinLength - obj.CabinRadius*2), ...
    cos(theta)*obj.CabinRadius];

Xs(:,1) = Xs(:,1) + obj.CockpitLength;
GeomObj = cast.GeomObj(Name="Fuselage", Xs=Xs);

%% Class I Mass Estimate - Raymer
K_d  = 1.12;
K_Lg = 1.12;
M_dg = obj.MTOM * obj.Mf_TOC * SI.lb;
n_z  = 2.5 * 1.5;
D    = (2 * obj.CabinRadius) * SI.ft;
b_w  = obj.Span * SI.ft;

SweepQtrChord = real(acosd(0.75 .* obj.Mstar ./ obj.TLAR.M_c));
tr   = -0.0083 * SweepQtrChord + 0.4597;
K_ws =  0.75 * ((1 + 2*tr) / (1 + tr)) * (b_w / L_f) * tand(SweepQtrChord);

S_f = pi*D*(obj.CabinLength.*SI.ft) + ...
    pi*(obj.CabinLength.*SI.ft)*(obj.CabinRadius.*SI.ft) + ...
    pi*(1.48*obj.CabinRadius.*SI.ft)*(obj.CabinRadius.*SI.ft);

W_fus_lb = 0.3280 * K_d * K_Lg * sqrt(M_dg * n_z) * (L_f^0.25) * (S_f^0.302) ...
           * ((1 + K_ws)^0.04) * ((L_f / D)^0.10);

m_fus_classI = W_fus_lb / SI.lb;

%% Class II.5 - Physics Based Beam Model
% Thin-walled tube

SF_ult = 1.5;
r      = obj.CabinRadius;
xw     = obj.WingPos;
xt     = obj.HtpPos;

if isprop(obj,'VtpPos') && ~isempty(obj.VtpPos)
    xv = obj.VtpPos;
else
    xv = 0.82 * L_f;
end

b = obj.Span;

% Material properties - Al 2024-T3
sigma_allow = 150e6;
tau_allow   = 0.58 * sigma_allow;
rho_mat     = 2800;
E_mat       = 70e9;
nu_mat      = 0.33;
G_mat       = E_mat / (2*(1+nu_mat));

% Structural parameters
dP_cabin    = 55e3;
t_min       = 0.001;
frame_pitch = 0.60;
A_frame     = 1.2e-3;
K_join      = 1.10;
k_comp      = 4.0;
k_shear     = 5.34;

% Cruise conditions ISA 35000 ft
rho_cruise = 0.379;
V_cruise   = obj.TLAR.M_c * sqrt(1.4*287*218.8);
V_EAS      = V_cruise * sqrt(rho_cruise/1.225);

if isprop(obj,'WingArea') && ~isempty(obj.WingArea)
    S_w = obj.WingArea;
else
    S_w = obj.MTOM * 9.81 / max(obj.WingLoading, 1);
end

if isprop(obj,'AR') && ~isempty(obj.AR)
    AR_w = obj.AR;
else
    AR_w = b^2 / S_w;
end
a_w = 2*pi*AR_w / (AR_w + 2);

if isprop(obj,'VtpArea') && ~isempty(obj.VtpArea)
    S_vtp = obj.VtpArea;
else
    S_vtp = 0.12 * S_w;
end

if isprop(obj,'Thrust') && ~isempty(obj.Thrust)
    T_eng = obj.Thrust;
else
    T_eng = 0;
end

W      = obj.MTOM * 9.81;
w_base = W / L_f;
Nx     = 300;
xvec   = linspace(0, L_f, Nx);

%% Load Cases

% Symmetric manoeuvre - 1g, 2.5g, -1g (CS-25)
n_man = [1.0, 2.5, -1.0];
V_man = zeros(3,Nx);
M_man = zeros(3,Nx);
for i = 1:3
    [V_man(i,:), M_man(i,:)] = beamCase(n_man(i), w_base, L_f, xw, xt, xvec);
end

% Antisymmetric torsion - 100:80 split of 2.5g
% Torsion acts between wing and VTP only - VTP reacts torque
[~, ~, Lw_25g, ~] = beamCase(2.5, w_base, L_f, xw, xt, xvec);
dLw       = 0.20 * Lw_25g / 2;
y_c       = 0.35 * (b/2);
T_antisym = dLw * y_c;
T_fus     = zeros(1,Nx);
T_fus(xvec >= xw & xvec <= xv) = T_antisym;

% Vertical gust - sharp-edged model
Kg        = 0.75;
WoS       = W / S_w;
U_gusts   = [20, 16, 8];
n_gust_up = zeros(1,3);
for i = 1:3
    dn = (rho_cruise*V_EAS*U_gusts(i)*a_w*Kg) / (2*WoS);
    n_gust_up(i) = 1 + dn;
end
[~, ig]    = max(n_gust_up);
n_gust_gov = n_gust_up(ig);
[V_gust, M_gust] = beamCase(n_gust_gov, w_base, L_f, xw, xt, xvec);

% Lateral gust on VTP
a_v    = 2*pi*1.5 / (1.5+2);
U_lat  = 15;
dF_vtp = 0.5 * rho_cruise * V_cruise * a_v * S_vtp * U_lat;
M_lat  = zeros(1,Nx);
for i = 1:Nx
    if xvec(i) <= xv
        M_lat(i) = dF_vtp * (xv - xvec(i));
    end
end

% Landing - 3 m/s sink rate, n=2.0
n_land    = 2.0;
M_land_kg = obj.MTOM * obj.Mf_Ldg;
w_land    = M_land_kg * 9.81 / L_f;
[V_land, M_land] = beamCase(n_land, w_land, L_f, xw, xt, xvec);

% Engine out - OEI yaw moment reacted at VTP
y_eng = 5.0;
if abs(xv - xw) > 1e-9
    F_vtp_oei = T_eng * y_eng / (xv - xw);
else
    F_vtp_oei = 0;
end
M_oei = zeros(1,Nx);
for i = 1:Nx
    if xvec(i) <= xv
        M_oei(i) = F_vtp_oei * (xv - xvec(i));
    end
end

%% Governing Load Envelopes
% Ultimate loads per CS-25, resultant moment per Megson Ch.9
M_gov_vert = max(abs([M_man; M_gust; M_land]), [], 1) * SF_ult;
V_gov_vert = max(abs([V_man; V_gust; V_land]), [], 1) * SF_ult;
M_gov_lat  = max(abs([M_lat; M_oei]),           [], 1) * SF_ult;
T_gov      = T_fus * SF_ult;
M_comb     = sqrt(M_gov_vert.^2 + M_gov_lat.^2);

%% Skin Thickness Sizing - Station by Station
% Pressure stresses included per Megson Ch.9
stringer_pitch = 0.18;
t_req = zeros(1,Nx);
for i = 1:Nx
    t = max(dP_cabin*r/sigma_allow, t_min);
    for iter = 1:40
        sigma_hoop     = dP_cabin * r / t;
        sigma_axial_p  = dP_cabin * r / (2*t);
        sigma_bend     = M_comb(i) / (pi*r^2*t);
        sigma_net_comp = max(sigma_bend - sigma_axial_p, 0);
        sigma_net_tens = sigma_bend + sigma_axial_p;
        tau_shear      = V_gov_vert(i) / (2*pi*r*t);
        tau_torsion    = T_gov(i)      / (2*pi*r^2*t);
        tau_total      = tau_shear + tau_torsion;
        sigma_cr = k_comp  * (pi^2*E_mat/(12*(1-nu_mat^2))) * (t/stringer_pitch)^2;
        tau_cr   = k_shear * (pi^2*E_mat/(12*(1-nu_mat^2))) * (t/stringer_pitch)^2;
        ok_strength = (sigma_net_tens <= sigma_allow) && ...
                      (sigma_hoop     <= sigma_allow) && ...
                      (tau_total      <= tau_allow);
        ok_buckle   = (sigma_net_comp/max(sigma_cr,1) + ...
                      (tau_total/max(tau_cr,1))^2) <= 1.0;
        if ok_strength && ok_buckle
            break
        end
        t = t * 1.05;
    end
    t_req(i) = min(t, 0.005);
end
t_avg = mean(t_req);

n_capped = sum(t_req >= 0.005);
if n_capped > 0
    fprintf('  Note: skin cap active at %d/%d stations\n', n_capped, Nx);
end

%% Stringer Sizing
sigma_comp = 0.5 * sigma_allow;
b_stringer = t_avg * sqrt((k_comp * pi^2 * E_mat) / ...
             (12 * (1 - nu_mat^2) * sigma_comp));
b_stringer = max(b_stringer, 0.10);
b_stringer = min(b_stringer, 0.30);
N_str      = max(8, ceil(2*pi*r / b_stringer));

Mmax       = max(M_comb);
I_req      = Mmax * r / sigma_allow;
I_skin     = pi * r^3 * t_avg;
I_rem      = max(I_req - 0.6*I_skin, 0);
A_stringer = max((2 * I_rem / r^2) / N_str, 1e-4);
A_stringer = min(A_stringer, 1500e-6);

%% Class II.5 Structural Mass
N_frames    = ceil(L_f / frame_pitch) + 1;
m_skin_25   = rho_mat * (2*pi*r) * L_f * t_avg;
m_frames_25 = rho_mat * (2*pi*r) * A_frame * N_frames;
m_str_25    = rho_mat * L_f * A_stringer * N_str;
m_fus_25    = K_join * (m_skin_25 + m_frames_25 + m_str_25);

%% Stiffness Distributions
EI_dist = E_mat * (pi*r^3.*t_req + 0.5*N_str*A_stringer*r^2);
GJ_dist = G_mat * (2*pi*r^3.*t_req);

%% Systems and Fuel System Mass
m_sys      = (270*(2*obj.CabinRadius) + 150) * L_f/9.81 * 2;
FuelMass   = obj.MTOM * obj.Mf_Fuel;
N_fuelTank = 3;
V_t        = FuelMass / cast.eng.Fuel.JA1.Density * SI.litre;
N_eng      = 2;
m_fuelsys  = 36.3*(N_eng+N_fuelTank-1) + 4.366*N_fuelTank^0.5*V_t^(1/3);

%% Centre Fuselage Fuel Tank
% Wing tanks hold 85% of total fuel (consistent with wing.m)
% Remaining 15% stored in belly tank between wing frames
rho_fuel      = 800;
m_wing_fuel   = 0.85 * FuelMass;
m_centre_fuel = 0.15 * FuelMass;

L_ctank     = 0.5 * (xt - xw);
W_ctank     = 1.2 * r;
H_ctank     = 0.5 * r;
V_ctank     = L_ctank * W_ctank * H_ctank;
m_ctank_cap = V_ctank * rho_fuel;

%% Force Balance - Major Attachment Frames
[~, ~, Lw_25g_ult,  Lt_25g_ult]  = beamCase(2.5,        w_base*SF_ult, L_f, xw, xt, xvec);
[~, ~, Lw_gust_ult, Lt_gust_ult] = beamCase(n_gust_gov, w_base*SF_ult, L_f, xw, xt, xvec);
[~, ~, Lw_land_ult, Lt_land_ult] = beamCase(n_land,     w_land*SF_ult, L_f, xw, xt, xvec);

Lw_ult = max(abs([Lw_25g_ult, Lw_gust_ult, Lw_land_ult]));
Lt_ult = max(abs([Lt_25g_ult, Lt_gust_ult, Lt_land_ult]));
Fv_ult = max(abs([dF_vtp, F_vtp_oei])) * SF_ult;

R_mlg  = W * 0.46;   % assumed static ground split, 92% on mains
R_nlg  = W * 0.08;

%% Results
fprintf('\n=== FUSELAGE STRUCTURAL SIZING ===\n');
fprintf('Fuselage length    : %.1f m\n', L_f);
fprintf('Cabin diameter     : %.1f m\n', 2*r);
fprintf('Fineness ratio     : %.2f\n',   L_f/(2*r));

fprintf('\n--- Class I (Raymer) ---\n');
fprintf('  Mass             : %.0f kg (%.2f%% MTOM)\n', m_fus_classI, 100*m_fus_classI/obj.MTOM);

fprintf('\n--- Class II.5 (Beam Model) ---\n');
fprintf('  Governing gust n : %.2f\n',     n_gust_up(ig));
fprintf('  Peak torque      : %.2f MNm\n', T_antisym/1e6);
fprintf('  Mean skin t      : %.2f mm\n',  t_avg*1e3);
fprintf('  N stringers      : %d\n',       N_str);
fprintf('  Stringer area    : %.1f mm2\n', A_stringer*1e6);
fprintf('  Skin mass        : %.0f kg\n',  m_skin_25);
fprintf('  Frame mass       : %.0f kg\n',  m_frames_25);
fprintf('  Stringer mass    : %.0f kg\n',  m_str_25);
fprintf('  Class II.5 mass  : %.0f kg (%.2f%% MTOM)\n', m_fus_25, 100*m_fus_25/obj.MTOM);
fprintf('  vs Class I       : %+.1f%%\n',  100*(m_fus_25-m_fus_classI)/m_fus_classI);

fprintf('\n--- Centre Fuel Tank ---\n');
fprintf('  Total fuel       : %.0f kg\n', FuelMass);
fprintf('  Wing tanks (85%%): %.0f kg\n', m_wing_fuel);
fprintf('  Centre tank req  : %.0f kg\n', m_centre_fuel);
fprintf('  Centre tank vol  : %.1f m3\n', V_ctank);
fprintf('  Centre tank cap  : %.0f kg\n', m_ctank_cap);
if m_ctank_cap >= m_centre_fuel
    fprintf('  Tank margin      : %+.1f%%\n', 100*(m_ctank_cap/m_centre_fuel - 1));
else
    fprintf('  WARNING: tank undersized by %.0f kg\n', m_centre_fuel - m_ctank_cap);
end

fprintf('\n--- Force Balance (Ultimate Loads, CS-25) ---\n');
fprintf('  Note: gear reactions are assumed static split, not solved\n');
fprintf('  Wing frame       : %.1f kN\n', Lw_ult/1e3);
fprintf('  HTP frame        : %.1f kN\n', Lt_ult/1e3);
fprintf('  VTP frame        : %.1f kN\n', Fv_ult/1e3);
fprintf('  Main gear/side   : %.1f kN\n', R_mlg/1e3);
fprintf('  Nose gear        : %.1f kN\n', R_nlg/1e3);
fprintf('==================================\n\n');

%% Mass Objects - Class II.5 feeds MDO
massObj(1) = cast.MassObj(Name="Fuselage",     m=m_fus_25,  X=[L_f/2; 0]);
massObj(2) = cast.MassObj(Name="Systems",      m=m_sys,     X=[L_f/2; 0]);
massObj(3) = cast.MassObj(Name="Fuel Systems", m=m_fuelsys, X=[L_f/2; 0]);

%% Store Distributions on ADP
obj.Fus_x      = xvec;
obj.Fus_M_vert = M_gov_vert;
obj.Fus_V_vert = V_gov_vert;
obj.Fus_T      = T_gov;
obj.Fus_t_req  = t_req;
obj.Fus_EI     = EI_dist;
obj.Fus_GJ     = GJ_dist;

end

%% Beam Model Helper Function
% Simple beam with uniform distributed load and point reactions at wing and HTP
% Returns shear force V, bending moment M, and attachment reactions Lw, Lt
function [V, M, Lw, Lt] = beamCase(n, w_base, L_f, xw, xt, xvec)
    w    = n * w_base;
    Wtot = w * L_f;
    Lt   = Wtot * (L_f/2 - xw) / (xt - xw);
    Lw   = Wtot + Lt;
    V    = -w.*xvec + Lw.*(xvec >= xw) - Lt.*(xvec >= xt);
    M    = -0.5*w.*xvec.^2 + Lw.*max(xvec-xw,0) - Lt.*max(xvec-xt,0);
end