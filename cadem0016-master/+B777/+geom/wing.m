function [GeomObj, massObj] = wing(obj)
% wing.m
% Class I wing geometry + mass model (Raymer-based)
% Supports baseline and folding-wing configurations
%
% Baseline: pure Raymer Class I
% Folding : first-order span + hinge mass penalties

%% ============================================================
%                 PLANFORM GEOMETRY
% =============================================================

% Quarter-chord sweep estimate (transonic transport)
SweepQtrChord = real(acosd(0.75 .* obj.Mstar ./ obj.TLAR.M_c));

% Empirical taper ratio (Raymer-style)
tr = -0.0083 * SweepQtrChord + 0.4597;

b = obj.Span;
S = (obj.MTOM * 9.81) / obj.WingLoading;

R_f = obj.CabinRadius;
L2 = obj.KinkPos - R_f;       % fuselage to kink
L3 = b/2 - obj.KinkPos;       % kink to tip

% Initial chord guess
c_r_star = (S/b) / (1 + tr);
c0 = (1 - (1-tr)*obj.KinkPos/(b/2)) * c_r_star;

% Solve for chord that gives correct area
c = fminsearch(@(x)(get_areas(x,L2,L3,R_f,tr,SweepQtrChord) - S).^2, c0);
[~, c_t, c_r, A1, A2, A3] = get_areas(c,L2,L3,R_f,tr,SweepQtrChord);

%% ============================================================
%                 MEAN AERODYNAMIC CHORD
% =============================================================

lambda = tr;

c_mac = (2/3) * c_r * (1 + lambda + lambda^2) / (1 + lambda);

% store in aircraft design parameters
obj.c_ac = c_mac;

%% ============================================================
%                 PLANFORM POINTS
% =============================================================

ys = [-b/2 -obj.KinkPos -R_f 0 R_f obj.KinkPos b/2]';
cs = [c_t c c_r c_r c_r c c_t]';

sweepLE   = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);
sweepHalf = atand((tand(SweepQtrChord)*L3 - c/4 + c_t/4)/L3);

x_le = [ ...
    tand(sweepLE)*(L2+L3)
    tand(sweepLE)*L2
    0
    0
    0
    tand(sweepLE)*L2
    tand(sweepLE)*(L2+L3) ];

x_le = x_le - 0.25*c_r;
x_te = x_le + cs;

Xs = [x_le ys; flipud(x_te) flipud(ys)];

% shift wing along fuselage
Xs(:,1) = Xs(:,1) + obj.WingPos;

GeomObj = cast.GeomObj(Name="Wing", Xs=Xs);

%% ============================================================
%                 CLASS I WING MASS (RAYMER)
% =============================================================

b_w = obj.Span * SI.ft;               % span [ft]
S_w = obj.WingArea * SI.ft^2;         % area [ft^2]
t_w = 0.15 * c_r * SI.ft;             % max thickness at root [ft]

% IMPORTANT: use half-chord sweep (matches reference code)
cosLambda = cosd(sweepHalf);

% Design gross weight [lb]
Wdg_lb = obj.MTOM * obj.Mf_TOC * SI.lb;

% Ultimate load factor
n_z = 2.5 * 1.5;

% Raymer wing weight [lb]
w_wing = 0.00125 * Wdg_lb * (b_w/cosLambda)^0.75 * ...
    (1 + sqrt(6.3*cosLambda/b_w)) * n_z^0.55 * ...
    (b_w*S_w/(t_w*Wdg_lb*cosLambda))^0.3;

% Baseline wing mass [kg]
m_wing_base = w_wing / SI.lb;

%% ============================================================
%                 FOLDING-WING CORRECTION (CLASS I)
% =============================================================

m_wing = m_wing_base;

if isfield(obj,'WingConfig') && obj.WingConfig == "folding"

    % k_span  : global bending-induced mass increase
    % k_hinge : local hinge + reinforcement mass
    m_wing = m_wing_base * (1 + obj.k_span + obj.k_hinge);

end

%% ============================================================
%                 MASS OBJECT
% =============================================================

massObj = cast.MassObj( ...
    Name = "Wing", ...
    m    = m_wing, ...
    X    = [obj.WingPos; 0] );

end

%% ============================================================
%                 HELPER FUNCTION
% =============================================================

function [S,c_t,c_r,A1,A2,A3] = get_areas(c,L2,L3,R_f,tr,SweepQtrChord)

c_t = tr * c;

sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);
c_r = c + tand(sweepLE)*L2;

A1 = c_r * R_f;
A2 = (c_r + c)/2 * L2;
A3 = (c + c_t)/2 * L3;

S = 2*(A1 + A2 + A3);

end
