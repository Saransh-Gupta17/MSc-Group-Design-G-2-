function [GeomObj,massObj] = empenage(obj)

%% =========================
% CRUISE CONDITION
%% =========================
M_c = obj.TLAR.M_c;
[rho,a] = cast.atmos(obj.TLAR.Alt_cruise);
q_c = 0.5*rho*(M_c*a)^2;

%% =========================
% ENSURE MAC EXISTS
%% =========================
if isempty(obj.c_ac)
    obj.c_ac = obj.c_mac;
end

%% =========================
% HTP AREA
%% =========================
L_h = obj.HtpPos - obj.WingPos;
obj.HtpArea = obj.V_HT * obj.WingArea * obj.c_ac / L_h;

%% =========================
% VTP AREA
%% =========================
obj.VtpArea = obj.WingArea * obj.Span * obj.V_VT / (obj.VtpPos - obj.WingPos);

%% =========================
% PRINT
%% =========================
fprintf('\n=== EMPENNAGE SIZING ===\n');
fprintf('HTP Area: %.2f m^2\n', obj.HtpArea);
fprintf('VTP Area: %.2f m^2\n', obj.VtpArea);

%% =========================
% HTP GEOMETRY
%% =========================
AR = 5;
SweepQtrChord = real(acosd(0.75*obj.Mstar/obj.TLAR.M_c));
tr = -0.0083*SweepQtrChord + 0.4597;

b = sqrt(AR*obj.HtpArea);
c_rh = obj.HtpArea/((1+tr)/2*b);

ys = [-b/2;0;b/2];
cs = [tr;1;tr]*c_rh;

x_le = -0.25*c_rh + [tand(SweepQtrChord)*(b/2);0;tand(SweepQtrChord)*(b/2)];
x_te = cs + x_le;

Xs = [x_le,ys;flipud(x_te),flipud(ys)];
Xs(:,1) = Xs(:,1) + obj.HtpPos;

GeomObj = cast.GeomObj(Name="HTP",Xs=Xs);

%% =========================
% HTP MASS
%% =========================
M_dg = obj.MTOM*obj.Mf_TOC*SI.lb;
tcr = 0.15; tct = 0.12;

m_HT = 0.016*(1.5*2.5*M_dg)^0.414 ...
*(SI.lb/SI.ft^2*q_c)^0.168 ...
*(obj.HtpArea*SI.ft^2)^0.896 ...
*(100*(tcr+tct)/2/cosd(SweepQtrChord))^-0.12 ...
*(AR/cosd(SweepQtrChord)^2)^0.043 ...
*tr^-0.02;

m_HT = m_HT/SI.lb;
massObj = cast.MassObj(Name="HTP",m=m_HT,X=[obj.HtpPos+c_rh*0.25;0]);

%% =========================
% VTP GEOMETRY
%% =========================
AR = 3.1;
b_VT = sqrt(AR*obj.VtpArea*2)/2;

c_rv = obj.VtpArea/(b_VT*(1+tr)/2);

ys = [0;b_VT];
cs = [1;tr]*c_rv;

x_qtr = [0;tand(SweepQtrChord)*b_VT];
x_le = -0.25*cs + x_qtr;
x_te = 0.75*cs + x_qtr;

Xs = [x_le,ys;flipud(x_te),flipud(ys)];
Xs(:,1) = Xs(:,1) + obj.VtpPos;

GeomObj(end+1) = cast.GeomObj(Name="VTP",Xs=Xs);

%% =========================
% VTP MASS
%% =========================
m_VT = 0.073*(1+0.2*0) ...
*(1.5*2.5*M_dg)^0.376 ...
*(SI.lb/SI.ft^2*q_c)^0.122 ...
*(obj.VtpArea*SI.ft^2)^0.873 ...
*(100*(tcr+tct)/2/cosd(SweepQtrChord))^-0.49 ...
*(AR/cosd(SweepQtrChord)^2)^0.357 ...
*tr^0.039;

m_VT = m_VT/SI.lb;
massObj(end+1) = cast.MassObj(Name="VTP",m=m_VT,X=[obj.VtpPos+c_rv*0.25;0]);

%% =========================
% CONTROL SURFACES
%% =========================
obj.ElevatorArea = 0.30 * obj.HtpArea;
obj.RudderArea   = 0.35 * obj.VtpArea;

fprintf('Elevator Area: %.2f m^2\n', obj.ElevatorArea);
fprintf('Rudder Area  : %.2f m^2\n', obj.RudderArea);

%% =========================
% ELEVATOR GEOMETRY
%% =========================
e_span = 0.9 * b;
e_chord = 0.3 * c_rh;

y_e = [-e_span/2; e_span/2];
x_e = obj.HtpPos + 0.7*c_rh;

Xs_e = [
    x_e, y_e(1);
    x_e+e_chord, y_e(1);
    x_e+e_chord, y_e(2);
    x_e, y_e(2)
];

GeomObj(end+1) = cast.GeomObj(Name="Elevator", Xs=Xs_e);

%% =========================
% RUDDER GEOMETRY
%% =========================
r_span = 0.9 * b_VT;
r_chord = 0.3 * c_rv;

y_r = [0; r_span];
x_r = obj.VtpPos + 0.7*c_rv;

Xs_r = [
    x_r, y_r(1);
    x_r+r_chord, y_r(1);
    x_r+r_chord, y_r(2);
    x_r, y_r(2)
];

GeomObj(end+1) = cast.GeomObj(Name="Rudder", Xs=Xs_r);

end