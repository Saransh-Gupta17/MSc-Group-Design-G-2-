function [GeomObj, massObj] = wing(obj)

%Step 1: Planform geometry 
fuel_mass_in_wing = 0.85 * obj.Mf_Fuel * obj.MTOM;

SweepQtrChord = real(acosd(0.75 .* obj.Mstar ./ obj.TLAR.M_c));

tr = -0.0083 * SweepQtrChord + 0.4597;

span = obj.Span;

% Compute wing area from loading
S = (obj.MTOM * 9.81) / obj.WingLoading;
obj.WingArea = S;

R_f = obj.CabinRadius;
L2 = obj.KinkPos - R_f;
L3 = span/2 - obj.KinkPos;

c_r_star = (S/span) / (1 + tr);
c0 = (1 - (1-tr)*obj.KinkPos/(span/2)) * c_r_star;

c = fminsearch(@(x)(get_areas(x,L2,L3,R_f,tr,SweepQtrChord) - S).^2, c0);
[~, c_t, c_r] = get_areas(c, L2, L3, R_f, tr, SweepQtrChord);

% MAC
c_mac = (2/3) * c_r * (1 + tr + tr^2) / (1 + tr);
obj.c_ac = c_mac;

%Geometry points for aircraft_sizer code

ys = [-span/2; -obj.KinkPos; -R_f; 0; R_f; obj.KinkPos; span/2];
cs = [c_t; c; c_r; c_r; c_r; c; c_t];

sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);

x_le = [ ...
    tand(sweepLE)*(L2+L3);
    tand(sweepLE)*L2;
    0;
    0;
    0;
    tand(sweepLE)*L2;
    tand(sweepLE)*(L2+L3) ];

x_le = x_le - 0.25*c_r;
x_te = x_le + cs;

% Closed 2D polygon
Xs = [x_le ys;
      flipud(x_te) flipud(ys)];

Xs = [Xs; Xs(1,:)];
Xs(:,1) = Xs(:,1) + obj.WingPos;

GeomObj = cast.GeomObj(Name="Wing", Xs=Xs);


% Aluminum 7075-T73 (from standards)
rho = 2810;              % kg/m³
E = 72e9;                % Pa
G = 27e9;                % Pa
sigma_y = 310e6;         % Pa
SF = 1.5;                % Safety factor

% Knockdown factors
knockdown_local = 0.75;
knockdown_interact = 0.90;

sigma_allow = sigma_y / SF * knockdown_local * knockdown_interact;

%             STEP 3: GET ENGINE MASS FROM ADP (Still not working)

engine_mass = 9300; % kg (one engine)

% Engine position: standard at 40% of half-span (pylon location)
% This is typical for transport aircraft
engine_pos = 0.4 * span/2;


%             STEP 4: LOAD CASES (Standard values from the finton documentation)


% Standard load cases for transport aircraft (CS-25 / CS-23)
% Values are industry-standard g-loads
n_cases = [2.5, 3.75, 2.0];  % [cruise gust, maneuver, landing]

%Iterative solver is a loop which keeps computing till the residual is
%required

g = 9.81;
MTOM = obj.MTOM;

% Structural sizing iteration
N = 400;
y = linspace(0, span/2, N);

wing_mass = 0.12 * MTOM;
tol = 0.01;
iter_max = 20;

for iter = 1:iter_max
    
    M_roots = zeros(size(n_cases));
    M_all = cell(size(n_cases));
    
    for i = 1:length(n_cases)
        
        n_load = n_cases(i);
        
        % Lift equilibrium (fuel removed from lumped term now)
        L = n_load * (MTOM - wing_mass - engine_mass - fuel_mass_in_wing) * g;

        % Elliptical lift distribution
        q0 = (4 * L) / (pi * span);
        q = q0 * sqrt(1 - (2*y/span).^2);

        % Wing self-weight
        w_wing = (wing_mass * g) / (span/2);

%DISTRIBUTED FUEL MODEL 

        % Total fuel in ONE wing (half-span)
        fuel_mass_half = 0.5 * fuel_mass_in_wing;
 
        % Fuel distribution shape (tank region ~90% semi-span)
        fuel_shape = sqrt(1 - (y / (0.9*span/2)).^2);
        fuel_shape(y > 0.9*span/2) = 0;

        % Normalize distribution
        fuel_shape = fuel_shape / trapz(y, fuel_shape);

        % Convert to distributed load (N/m)
        q_fuel = fuel_mass_half * g * fuel_shape;

        % Apply all distributed loads
        q = q - w_wing - q_fuel;
        
        % Engine load
        F_engine = n_load * engine_mass * g;
        [~, idx_eng] = min(abs(y - engine_pos));
        
        % Shear
        V = cumtrapz(flip(y), flip(q));
        V = flip(V);
        V(idx_eng:end) = V(idx_eng:end) - F_engine;
        
        % Bending
        M = cumtrapz(flip(y), flip(V));
        M = flip(M);
        
        % Store results
        M_roots(i) = M(1);
        M_all{i} = M;
        
    end
    
    % Select worst case
    [~, idx_max] = max(M_roots);
    M = M_all{idx_max};   % correct spanwise bending
    M_design = M(1);
    
    %% STRUCTURAL SIZING BASED ON BENDING MOMENT
    
    c_root = 2 * obj.WingArea / (span * (1 + tr));
    
    box_height   = 0.12 * c_root;
    flange_width = 0.40 * c_root;
    
    Z_req = M_design / sigma_allow;
    
    A_skin_root = Z_req / box_height;
    t_root = A_skin_root / (2 * flange_width);
    
    t_span = t_root * sqrt(1 - (2*y/span).^2);
    t_span = max(t_span, 1e-4);   % prevent zero thickness
    
    skin_volume = 2 * trapz(y, flange_width .* t_span);
    
    %% ---- SPAR ----
    
    t_spar_root = 0.02 * c_root;
    t_spar_tip  = 0.005 * c_root;
    
    t_spar_span = linspace(t_spar_root, t_spar_tip, N);
    spar_volume = 2 * trapz(y, box_height .* t_spar_span);
    
    %% ---- RIBS (Buckling-based) ----
    
    k = 4;
    
    Z_span = (box_height.^2 .* flange_width .* t_span) / 2;
    Z_span = max(Z_span, 1e-6);   % prevent divide-by-zero
    
    stress_span = M ./ Z_span;

    %  remove NaN/Inf
    stress_span(~isfinite(stress_span)) = 0;

    % enforce minimum stress
    stress_span = max(stress_span, 1e5);   % 0.1 MPa floor
    
    % numerical safety
    stress_span(stress_span <= 1e3) = 1e3;
    
    rib_spacing_span = sqrt((pi^2 * E .* t_span.^2) ./ (k .* stress_span));
    rib_spacing_span = max(min(rib_spacing_span, 1.0), 0.3);
    
    n_ribs_half = trapz(y, 1 ./ rib_spacing_span);
    n_ribs = 2 * n_ribs_half;
    
    rib_area_span = flange_width .* box_height .* sqrt(1 - (2*y/span).^2);
    rib_thickness_span = 0.002 * sqrt(stress_span / max(stress_span));
    rib_thickness_span = max(rib_thickness_span, 0.0015);  % minimum thickness
    
    rib_volume = 2 * trapz(y, rib_area_span .* rib_thickness_span ./ rib_spacing_span);
    
    %% ---- TOTAL ----
    
    structure_volume = skin_volume + spar_volume + rib_volume;
    structure_mass = rho * structure_volume;
    
    control_surface_mass = 0.06 * structure_mass;
    
    new_wing_mass = structure_mass + control_surface_mass;
    
    %% ---- CONVERGENCE ----
    
    residual = abs(new_wing_mass - wing_mass) / wing_mass;
    wing_mass = 0.7 * wing_mass + 0.3 * new_wing_mass;
    
    if residual < tol
        break
    end
    
end
m_wing = wing_mass;
% Store final distributions for plotting
V_final = V;
M_final = M;

% Flexural rigidity (EI)
Z_span = (box_height.^2 .* flange_width .* t_span) / 2;
EI_span = E .* Z_span;


%MARGIN CHECKS

stress_design = M_design / Z_req;
margin_static = (sigma_allow / stress_design) - 1;

% Buckling check
I_spar = (box_height^3 * flange_width) / 12;
L_eff = 0.7 * span/2;
sigma_crit = (pi^2 * E) / (L_eff / sqrt(box_height))^2;
sigma_crit = sigma_crit * 0.85;

margin_buckling = (sigma_crit / stress_design) - 1;


%             DIAGNOSTIC OUTPUT FOR THE FINAL REPORT 


fprintf('\n=== WING STRUCTURAL SIZING ===\n');
fprintf('MTOM: %.1f t\n', MTOM/1e3);
fprintf('Span: %.1f m\n', span);
fprintf('Wing Area: %.1f m²\n', S);
fprintf('Engine Mass: %.0f kg\n', engine_mass);
fprintf('\nStructural Sizing:\n');
fprintf('  Skin volume: %.4f m³\n', skin_volume);
fprintf('  Spar volume: %.4f m³\n', spar_volume);
fprintf('  Rib volume: %.4f m³\n', rib_volume);
fprintf('  Total structure: %.1f kg\n', structure_mass);
fprintf('  Control surfaces: %.1f kg\n', control_surface_mass);
fprintf('  TOTAL WING MASS: %.1f kg (%.2f%% MTOM)\n', m_wing, 100*m_wing/MTOM);
fprintf('\nMargins:\n');
fprintf('  Design stress: %.1f MPa\n', stress_design/1e6);
fprintf('  Allowable: %.1f MPa\n', sigma_allow/1e6);
fprintf('  Static margin: %+.1f%%\n', 100*margin_static);
fprintf('  Buckling margin: %+.1f%%\n', 100*margin_buckling);

if margin_static < 0 || margin_buckling < 0
    warning('Wing margins are NEGATIVE!');
end

fprintf('========================\n\n');

% Mass of the wing itself 


massObj = cast.MassObj( ...
    Name = "Wing_Structural", ...
    m    = m_wing, ...
    X    = [obj.WingPos; 0] );
    % Store distributions in ADP object for plotting
   obj.Wing_y  = y;
obj.Wing_V  = V_final;
obj.Wing_M  = M_final;
obj.Wing_EI = EI_span;

end

%% ============================================================
%                 HELPER FUNCTION
%% ============================================================

function [S, c_t, c_r, A1, A2, A3] = get_areas(c, L2, L3, R_f, tr, SweepQtrChord)
    c_t = tr * c;
    sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);
    c_r = c + tand(sweepLE) * L2;
    
    A1 = c_r * R_f;
    A2 = (c_r + c)/2 * L2;
    A3 = (c + c_t)/2 * L3;
    
    S = 2 * (A1 + A2 + A3);
end
