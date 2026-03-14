clear
clc

%% -------------------------------------------------
% CREATE TEST AIRCRAFT OBJECT
%% -------------------------------------------------

obj = struct();

% Wing geometry
obj.WingArea = 599.8;      % m^2
obj.Span     = 64.8;       % m
obj.c_ac     = 7.0;        % mean aerodynamic chord

% Aircraft layout
obj.WingPos  = 15;         % m
obj.HtpPos   = 30;         % m
obj.VtpPos   = 30;         % m

% Tail volume coefficients
obj.V_HT = 0.9;
obj.V_VT = 0.08;

% Flight conditions
obj.TLAR.M_c = 0.8;
obj.TLAR.Alt_cruise = 11000;

% Aerodynamic parameter
obj.Mstar = 0.93;

% Mass parameters
obj.MTOM = 374000;
obj.Mf_TOC = 0.97;

%% -------------------------------------------------
% RUN EMPENNAGE MODULE
%% -------------------------------------------------

[GeomObj, MassObj] = B777.geom.empenage(obj);

%% -------------------------------------------------
% DISPLAY RESULTS
%% -------------------------------------------------

fprintf('\n====================================\n')
fprintf('EMPENNAGE RESULTS\n')
fprintf('====================================\n')

% Horizontal tail
fprintf('\nComponent 1: %s\n', GeomObj(1).Name)
fprintf('Mass: %.2f kg\n', MassObj(1).m)

% Vertical tail
fprintf('\nComponent 2: %s\n', GeomObj(2).Name)
fprintf('Mass: %.2f kg\n', MassObj(2).m)

%% Show geometry structures

fprintf('\n------------------------------------\n')
fprintf('GEOMETRY STRUCTURES\n')
fprintf('------------------------------------\n')

disp(GeomObj)

fprintf('\n------------------------------------\n')
fprintf('MASS STRUCTURES\n')
fprintf('------------------------------------\n')

disp(MassObj)