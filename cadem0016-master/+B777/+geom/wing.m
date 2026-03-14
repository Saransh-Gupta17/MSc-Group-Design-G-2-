function [GeomObj, massObj] = wing(obj)


%% ---------------- Planform geometry ----------------

SweepQtrChord = real(acosd(0.75 .* obj.Mstar ./ obj.TLAR.M_c));

tr = -0.0083 * SweepQtrChord + 0.4597;

b = obj.Span;

S = obj.WingArea;

R_f = obj.CabinRadius;

L2 = obj.KinkPos - R_f;
L3 = b/2 - obj.KinkPos;

c_r_star = (S/b) / (1 + tr);

c0 = (1 - (1-tr)*obj.KinkPos/(b/2)) * c_r_star;

c = fminsearch(@(x)(get_areas(x,L2,L3,R_f,tr,SweepQtrChord) - S).^2, c0);

[~, c_t, c_r] = get_areas(c,L2,L3,R_f,tr,SweepQtrChord);
% Compute Mean Aerodynamic Chord (MAC)
lambda = c_t / c_r;
obj.c_ac = (2/3) * c_r * (1 + lambda + lambda^2) / (1 + lambda);

sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);

ys = [-b/2 -obj.KinkPos -R_f 0 R_f obj.KinkPos b/2]';

cs = [c_t c c_r c_r c_r c c_t]';

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

Xs(:,1) = Xs(:,1) + obj.WingPos;

GeomObj = cast.GeomObj(Name="Wing", Xs=Xs);

% CLASS II.5 STRUCTURAL WING MASS MODEL


span = obj.Span;
S = obj.WingArea;
MTOM = obj.MTOM;

g = 9.81;

%% Load cases

n_cases = [2.5 2.0 2.3]; % manoeuvre, landing, gust

W = MTOM*g;
L_cases = n_cases * W;

N = 200;
y = linspace(0,span/2,N);

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

%% Wing box geometry

lambda = 0.3;

c_root = 2*S/(span*(1+lambda));

box_height = 0.12*c_root;
flange_width = 0.4*c_root;

%% Material properties

sigma_allow = 250e6; % Pa
rho_al = 2800; % kg/m^3

%% Section modulus requirement

Z_req = M_design / sigma_allow;

A_skin_root = (2*Z_req)/box_height;

t_root = A_skin_root/(2*flange_width);

%% Spanwise thickness distribution

t_span = t_root * sqrt(1-(2*y/span).^2);

%% Skin volume

skin_volume = 2 * trapz(y, flange_width .* t_span);

%% Spar volume (tapered)

t_spar_root = 0.02;
t_spar_tip  = 0.005;

t_spar_span = linspace(t_spar_root,t_spar_tip,N);

spar_volume = 2 * trapz(y, box_height .* t_spar_span);

%% Rib estimation

rib_spacing = 0.6;
n_ribs = span / rib_spacing;

rib_area = flange_width * box_height;
rib_volume = n_ribs * rib_area * 0.0015;

%% Structural volume

structure_volume = skin_volume + spar_volume + rib_volume;

structure_mass = rho_al * structure_volume;

%% Control surface mass

control_surface_mass = 0.05 * structure_mass;

wing_mass = structure_mass + control_surface_mass;

%% Return wing mass

massObj = cast.MassObj(Name="Wing", m=wing_mass, X=[obj.WingPos; 0]);

end


%% -------- Wing area helper --------
function [S,c_t,c_r] = get_areas(c,L2,L3,R_f,tr,SweepQtrChord)

c_t = tr * c;

sweepLE = atand((tand(SweepQtrChord)*L3 + c/4 - c_t/4)/L3);

c_r = c + tand(sweepLE)*L2;

A1 = c_r * R_f;
A2 = (c_r + c)/2 * L2;
A3 = (c + c_t)/2 * L3;

S = 2*(A1 + A2 + A3);


end