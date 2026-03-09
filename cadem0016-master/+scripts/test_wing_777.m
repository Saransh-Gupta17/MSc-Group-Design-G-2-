clear; clc;

%% ================== Flight / Aero assumptions ==================

obj.TLAR.M_c = 0.8;     % Cruise Mach number (777-class)
obj.Mstar    = 0.93;     % Drag divergence Mach proxy

%% ================== Mass & loading assumptions ==================

obj.MTOM = 3.74e5;       % kg  (~428 t, Boeing 777F)
obj.WingLoading = 7000;  % N/m^2 (typical widebody)
obj.Mf_TOC = 0.95;       % Mass fraction at top-of-climb

%% ================== Geometry assumptions ==================

obj.Span = 70.3;         % m (baseline 777F span)
obj.KinkPos = 13.0;      % m (~40% of semi-span)
obj.CabinRadius = 3.6;   % m (~8.5 m fuselage diameter)
obj.WingPos = 20.0;      % m (wing AC position)

%% ================== Derived quantities ==================

obj.WingArea = (obj.MTOM * 9.81) / obj.WingLoading;

%% ================== Folding-wing configuration ==================

obj.WingConfig = "folding";   % activates hinge logic in wing.m

% --- Class I folding-wing penalties ---
obj.k_span  = 0.10;   % 10% global span-induced mass penalty
obj.k_hinge = 0.04;   % 4% local hinge/reinforcement penalty

%% ================== Run Class I wing model ==================

[GeomObj, MassObj] = B777.geom.wing(obj);

%% ================== Console output ==================

fprintf('\n=====================================================\n');
fprintf(' CLASS I FOLDING-WING MODEL (777-LIKE INPUTS)\n');
fprintf('=====================================================\n');
fprintf('MTOM                     : %.1f tonnes\n', obj.MTOM/1000);
fprintf('Wing span (baseline)     : %.1f m\n', obj.Span);
fprintf('Wing area                : %.1f m^2\n', obj.WingArea);
fprintf('Span penalty (k_span)    : %.1f %%\n', obj.k_span*100);
fprintf('Hinge penalty (k_hinge)  : %.1f %%\n', obj.k_hinge*100);
fprintf('Wing mass (total)        : %.1f tonnes\n', MassObj.m/1000);
fprintf('=====================================================\n\n');

%% ================== Plot geometry ==================

cast.draw(GeomObj, MassObj);
axis equal
title('Class I Folding Wing (777-class)')
