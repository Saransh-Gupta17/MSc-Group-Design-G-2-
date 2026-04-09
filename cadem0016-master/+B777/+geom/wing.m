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

%STEP 3: GET ENGINE MASS FROM ADP (Still not working)


engine_mass = obj.Engine.Mass;

%Engine position from ADP
engine_pos = obj.EngineLocation;


%STEP 4: LOAD CASES (Standard values from the finton documentation)


% Standard load cases for transport aircraft (CS-25 / CS-23)
% Values are industry-standard g-loads
n_cases = [2.5, 3.75, 2.0];  % [cruise gust, maneuver, landing]

%Iterative solver is a loop which keeps computing till the residual is
%required

g = 9.81;
MTOM = obj.MTOM;

% Structural sizing iteration
c_root = 2 * obj.WingArea / (span * (1 + tr));
N = 400;
y = linspace(0, span/2, N);

% ---- SPANWISE CHORD VARIATION ----
c_span = c_root * sqrt(1 - (2*y/span).^2);

% Prevent chord going to zero at tip (numerical stability)
c_span = max(c_span, 0.2 * c_root);

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

% -------------------- DISTRIBUTED FUEL MODEL --------------------

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
    
    %% ---- STRUCTURAL SIZING ----
    
    
    
    box_height_span   = 0.12 * c_span;
    flange_width_span = 0.40 * c_span;
    
    Z_req = M_design / sigma_allow;
    
    A_skin_root = Z_req / (0.12 * c_root);
    t_root = A_skin_root / (2 * (0.40 * c_root));
    
    t_span = t_root * sqrt(1 - (2*y/span).^2);
    t_span = max(t_span, 1e-4);   % prevent zero thickness
    
    skin_volume = 2 * trapz(y, flange_width_span .* t_span);
    
    %% ---- SPAR ----
    
    t_spar_root = 0.02 * c_root;
    t_spar_tip  = 0.005 * c_root;
    
    t_spar_span = linspace(t_spar_root, t_spar_tip, N);
    spar_volume = 2 * trapz(y, box_height_span .* t_spar_span);
    
    
    %% ---- RIBS (Buckling-based) ----
    
    k = 4;
    
    Z_span = (box_height_span.^2 .* flange_width_span .* t_span) / 2;
    Z_span = max(Z_span, 1e-6);   % prevent divide-by-zero
    
    stress_span = M ./ Z_span;

    %  remove NaN/Inf
    stress_span(~isfinite(stress_span)) = 0;

    % enforce minimum stress
    stress_span = max(stress_span, 1e5);   % 0.1 MPa floor
    
    % numerical safety
    stress_span(stress_span <= 1e3) = 1e3;
    
    rib_spacing_span = 0.52 * sqrt((pi^2 * E .* t_span.^2) ./ (k .* stress_span));

    % realistic bounds
    rib_spacing_span = min(rib_spacing_span, 1.3);
    rib_spacing_span = max(rib_spacing_span, 0.30);
    
    n_ribs_half = trapz(y, 1 ./ rib_spacing_span);
    n_ribs = 2 * n_ribs_half;
    
    rib_area_span = flange_width_span .* box_height_span;
    rib_thickness_span = 0.002 * sqrt(stress_span / max(stress_span));
    rib_thickness_span = max(rib_thickness_span, 0.0015);  % minimum thickness
    
    rib_volume = 2 * trapz(y, rib_area_span .* rib_thickness_span ./ rib_spacing_span);

    %% ---- FUEL TANK CHECK ----

    fuel_density = 800; % kg/m^3 (Jet-A1 assumption)

    % Assume tank occupies ~60% of wingbox width
    tank_width_span = 0.6 * flange_width_span;

    % Tank volume (integrated along span)
    tank_volume = 2 * trapz(y, tank_width_span .* box_height_span);

    % Required fuel volume
    fuel_volume_required = fuel_mass_in_wing / fuel_density;

    % Margin
    fuel_volume_margin = (tank_volume / fuel_volume_required) - 1;

    if fuel_volume_margin < 0
        warning('Fuel tank volume insufficient!');
    end
    
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
Z_span = (box_height_span.^2 .* flange_width_span .* t_span) / 2;
EI_span = E .* Z_span;

% ---- TORSIONAL STIFFNESS (GJ) ----

J_span = (2/3) * flange_width_span .* (box_height_span.^3);

GJ_span = G .* J_span;

% Store in ADP for plotting
obj.Wing_GJ = GJ_span;


%MARGIN CHECKS

stress_design = M_design / Z_req;
margin_static = (sigma_allow / stress_design) - 1;

% ---- CONSISTENT BUCKLING CHECK ----

sigma_crit_span = (k * pi^2 * E .* t_span.^2) ./ (rib_spacing_span.^2);

margin_buckling_span = (sigma_crit_span ./ stress_span) - 1;

margin_buckling = min(margin_buckling_span);

%% ---- CLASS I WING MASS (EMPIRICAL FRACTION) ----

wing_mass_fraction = 0.15;   % typical range: 0.12–0.18

W_wing_classI = wing_mass_fraction * MTOM;

mass_diff = (W_wing_classI / m_wing) - 1;



%             OUTPUT FOR THE FINAL REPORT 


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
fprintf('  Fuel tank volume margin: %+.1f%%\n', 100*fuel_volume_margin);
fprintf('\n=== CLASS I COMPARISON ===\n');
fprintf('  Class I wing mass: %.1f kg\n', W_wing_classI);
fprintf('  Class II.5 wing mass: %.1f kg\n', m_wing);
fprintf('  Difference: %+.1f%%\n', 100*mass_diff);

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
obj.Wing_GJ = GJ_span;

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