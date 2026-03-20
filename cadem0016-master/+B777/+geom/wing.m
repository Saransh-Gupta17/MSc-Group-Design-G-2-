function [GeomObj, massObj] = wing(obj)

%% ============================================================
%                 PLANFORM GEOMETRY
%% ============================================================

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

%% ============================================================
%                 GEOMETRY POINTS
%% ============================================================

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

Xs = [Xs; Xs(1,:)];      % explicitly close polygon
Xs(:,1) = Xs(:,1) + obj.WingPos;

GeomObj = cast.GeomObj(Name="Wing", Xs=Xs);

%% ============================================================
%                 CLASS II.5 STRUCTURAL MODEL
%% ============================================================

g = 9.81;
MTOM = obj.MTOM;

engine_mass = 0.023 * MTOM;
engine_pos  = 0.4 * span/2;

n_cases = [2.5 2.0 2.3];

N = 400;
y = linspace(0, span/2, N);

% Initial guess
wing_mass = 0.12 * MTOM;

for iter = 1:5

    M_roots = zeros(size(n_cases));

    for i = 1:length(n_cases)

        % Lift equilibrium
        L = n_cases(i) * (MTOM - wing_mass - engine_mass) * g;

        % Elliptical lift distribution
        q0 = (4 * L) / (pi * span);
        q  = q0 * sqrt(1 - (2*y/span).^2);

        % Wing self-weight distribution over half span
        w_wing = (wing_mass * g) / (span/2);
        q = q - w_wing;

        % Engine point load
        F_engine = n_cases(i) * engine_mass * g;
        [~, idx] = min(abs(y - engine_pos));

        % Shear
        V = cumtrapz(flip(y), flip(q));
        V = flip(V);
        V(idx:end) = V(idx:end) - F_engine;

        % Bending moment
        M = cumtrapz(flip(y), flip(V));
        M = flip(M);

        M_roots(i) = M(1);
    end

    M_design = max(M_roots);

    %% STRUCTURAL SIZING

    c_root = 2 * obj.WingArea / (span * (1 + tr));

    box_height   = 0.12 * c_root;
    flange_width = 0.4 * c_root;

    sigma_allow = 250e6;
    rho = 2800;

    Z_req = M_design / sigma_allow;

    A_skin_root = (2 * Z_req) / box_height;
    t_root = A_skin_root / (2 * flange_width);

    t_span = t_root * sqrt(1 - (2*y/span).^2);

    skin_volume = 2 * trapz(y, flange_width .* t_span);

    t_spar_root = 0.02 * c_root;
    t_spar_tip  = 0.005 * c_root;

    t_spar_span = linspace(t_spar_root, t_spar_tip, N);
    spar_volume = 2 * trapz(y, box_height .* t_spar_span);

    rib_spacing = 0.6;
    n_ribs = span / rib_spacing;

    rib_area = flange_width * box_height;
    rib_volume = n_ribs * rib_area * 0.0015;

    structure_volume = skin_volume + spar_volume + rib_volume;
    structure_mass = rho * structure_volume;

    control_surface_mass = 0.05 * structure_mass;

    new_wing_mass = structure_mass + control_surface_mass;

    wing_mass = 0.5 * wing_mass + 0.5 * new_wing_mass;
end

m_wing = wing_mass;

fprintf("DEBUG Final wing mass: %.2f tonnes\n", m_wing/1000);

%% ============================================================
%                 MASS OBJECT
%% ============================================================

massObj = cast.MassObj( ...
    Name = "Wing_Structural", ...
    m    = m_wing, ...
    X    = [obj.WingPos; 0] );

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