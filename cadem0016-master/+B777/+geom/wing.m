function [GeomObj, massObj] = wing(obj)
% wing - Class I baseline wing model using Raymer method
% Generates wing geometry and structural mass estimate

%% ---------------- Planform geometry ----------------

% Quarter-chord sweep estimation
SweepQtrChord = real(acosd(0.75 .* obj.Mstar ./ obj.TLAR.M_c));

% Taper ratio correlation
tr = -0.0083 * SweepQtrChord + 0.4597;

% Wing span
b = obj.Span;

% Wing area from wing loading
S = (obj.MTOM * 9.81) / obj.WingLoading;

% Cabin radius
R_f = obj.CabinRadius;

% Spanwise distances
L2 = obj.KinkPos - R_f;
L3 = b/2 - obj.KinkPos;

% Initial root chord guess
c_r_star = (S/b) / (1 + tr);
c0 = (1 - (1-tr)*obj.KinkPos/(b/2)) * c_r_star;

% Solve for chord using area matching
c = fminsearch(@(x)(get_areas(x,L2,L3,R_f,tr,SweepQtrChord) - S).^2, c0);

% Retrieve final chord values
[~, c_t, c_r] = get_areas(c,L2,L3,R_f,tr,SweepQtrChord);

% Leading edge sweep
sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);

% Spanwise stations
ys = [-b/2 -obj.KinkPos -R_f 0 R_f obj.KinkPos b/2]';

% Chord distribution
cs = [c_t c c_r c_r c_r c c_t]';

% Leading edge x locations
x_le = [ ...
    tand(sweepLE)*(L2+L3)
    tand(sweepLE)*L2
    0
    0
    0
    tand(sweepLE)*L2
    tand(sweepLE)*(L2+L3) ];

% Move reference to quarter chord
x_le = x_le - 0.25*c_r;

% Trailing edge
x_te = x_le + cs;

% Polygon coordinates
Xs = [x_le ys; flipud(x_te) flipud(ys)];

% Geometry object
GeomObj = cast.GeomObj(Name="Wing", Xs=Xs);

%% ---------------- Class I wing mass (Raymer) ----------------

% Convert to imperial units for Raymer equation
b_w = obj.Span * SI.ft;
S_w = S * SI.ft^2;

% Mean structural thickness estimate
t_w = 0.15 * c_r * SI.ft;

% Sweep correction
cosLambda = cosd(SweepQtrChord);

% Design weight in pounds
Wdg_lb = obj.MTOM * obj.Mf_TOC * SI.lb;

% Ultimate load factor
n_z = 2.5 * 1.5;

% Raymer wing weight equation
w_wing = 0.00125 * Wdg_lb * (b_w/cosLambda)^0.75 * ...
    (1 + sqrt(6.3*cosLambda/b_w)) * n_z^0.55 * ...
    (b_w*S_w/(t_w*Wdg_lb*cosLambda))^0.3;

% Convert back to kg
m_wing = w_wing / SI.lb;

% Mass object
massObj = cast.MassObj(Name="Wing", m=m_wing, X=[obj.WingPos; 0]);

end

%% -------- Wing area helper --------
function [S,c_t,c_r] = get_areas(c,L2,L3,R_f,tr,SweepQtrChord)

% Tip chord
c_t = tr * c;

% Leading edge sweep
sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);

% Root chord
c_r = c + tand(sweepLE)*L2;

% Areas of wing sections
A1 = c_r * R_f;
A2 = (c_r + c)/2 * L2;
A3 = (c + c_t)/2 * L3;

% Total wing area
S = 2*(A1 + A2 + A3);

end