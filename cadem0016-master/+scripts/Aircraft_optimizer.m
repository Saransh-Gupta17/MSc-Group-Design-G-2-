% ============================================================
% Disable ALL figures during optimisation
% ============================================================
oldState = get(0, 'DefaultFigureVisible');
set(0, 'DefaultFigureVisible', 'off');
cleanupObj = onCleanup(@() set(0, 'DefaultFigureVisible', oldState));
%% Optimize_PayloadAltitude_IntegerFleet
% Robust optimization using:
%   - integer fleet size (enumerated)
%   - continuous cruise altitude optimized by fmincon
%
% Payload is derived from fleet size:
%   Payload_per_aircraft = Total_Payload / Fleet_size
%
% Constraints:
%   60000 kg <= Payload <= 125000 kg
%   9000 m   <= cruise altitude <= 12000 m
%   wingspan <= 72 m
%
% Objectives:
%   1) Minimize Fleet DOC
%   2) Minimize ATR
%   3) Build approximate Pareto front using weighted sums

clear; clc; close all;

%% ============================================================
%% 1) Baseline model
%% ============================================================
base = makeBaselineADP();

payload_min = 60000;    % kg
payload_max = 125000;   % kg

alt_lb = 9000;          % m
alt_ub = 12000;         % m
alt0   = min(max(base.cruise_altitude, alt_lb), alt_ub);

TotalPayload = base.Total_Payload;

% Feasible integer fleet sizes implied by payload bounds
fleet_min = ceil(TotalPayload / payload_max);
fleet_max = floor(TotalPayload / payload_min);

fleetList = fleet_min:fleet_max;

if isempty(fleetList)
    error('No feasible integer fleet sizes exist for the requested payload bounds.');
end

fprintf('\n============================================================\n');
fprintf('Feasible integer fleet sizes from payload bounds:\n');
disp(fleetList);

for i = 1:numel(fleetList)
    fprintf('Fleet size = %d --> Payload/aircraft = %.1f kg\n', ...
        fleetList(i), TotalPayload / fleetList(i));
end

opts = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 3000, ...
    'MaxIterations', 150, ...
    'StepTolerance', 1e-8, ...
    'OptimalityTolerance', 1e-6, ...
    'ConstraintTolerance', 1e-6);

%% ============================================================
%% 2) Minimize Fleet DOC
%% ============================================================
DOC_results = struct([]);
k = 0;

fprintf('\n============================================================\n');
fprintf('MINIMIZING FLEET DOC\n');
fprintf('============================================================\n');

for FS = fleetList
    k = k + 1;

    objDOC = @(h) objectiveDOC(FS, h, base, payload_min, payload_max);

    [h_sol, f_sol, exitflag, output] = fmincon( ...
        objDOC, alt0, [], [], [], [], alt_lb, alt_ub, ...
        @(h) nonlinearConstraints(FS, h, base, payload_min, payload_max), opts);

    R = evaluateDesign(FS, h_sol, base, payload_min, payload_max);

    DOC_results(k).FleetSize      = FS;
    DOC_results(k).Payload        = R.Payload;
    DOC_results(k).Altitude_m     = h_sol;
    DOC_results(k).FleetDOC       = R.FleetDOC;
    DOC_results(k).ATR            = R.ATR_scalar;
    DOC_results(k).Span           = R.Span;
    DOC_results(k).MTOM           = R.MTOM;
    DOC_results(k).BlockFuel      = R.BlockFuel;
    DOC_results(k).isValid        = R.isValid;
    DOC_results(k).exitflag       = exitflag;
    DOC_results(k).output         = output;
    DOC_results(k).objective      = f_sol;
end

validDOC = find([DOC_results.isValid]);
[~, idxDocBestLocal] = min([DOC_results(validDOC).FleetDOC]);
DOC_best = DOC_results(validDOC(idxDocBestLocal));
validDOC = find([DOC_results.isValid]);
if isempty(validDOC)
    error('No valid DOC solutions found.');
end
[~, idxDocBestLocal] = min([DOC_results(validDOC).FleetDOC]);
DOC_best = DOC_results(validDOC(idxDocBestLocal));

validATR = find([ATR_results.isValid]);
if isempty(validATR)
    error('No valid ATR solutions found.');
end
[~, idxATRBestLocal] = min([ATR_results(validATR).ATR]);
ATR_best = ATR_results(validATR(idxATRBestLocal));
%% ============================================================
%% 3) Minimize ATR
%% ============================================================
ATR_results = struct([]);
k = 0;

fprintf('\n============================================================\n');
fprintf('MINIMIZING ATR\n');
fprintf('============================================================\n');

for FS = fleetList
    k = k + 1;

    objATR = @(h) objectiveATR(FS, h, base, payload_min, payload_max);

    [h_sol, f_sol, exitflag, output] = fmincon( ...
        objATR, alt0, [], [], [], [], alt_lb, alt_ub, ...
        @(h) nonlinearConstraints(FS, h, base, payload_min, payload_max), opts);

    R = evaluateDesign(FS, h_sol, base, payload_min, payload_max);

    ATR_results(k).FleetSize      = FS;
    ATR_results(k).Payload        = R.Payload;
    ATR_results(k).Altitude_m     = h_sol;
    ATR_results(k).FleetDOC       = R.FleetDOC;
    ATR_results(k).ATR            = R.ATR_scalar;
    ATR_results(k).Span           = R.Span;
    ATR_results(k).MTOM           = R.MTOM;
    ATR_results(k).BlockFuel      = R.BlockFuel;
    ATR_results(k).isValid        = R.isValid;
    ATR_results(k).exitflag       = exitflag;
    ATR_results(k).output         = output;
    ATR_results(k).objective      = f_sol;
end

validATR = find([ATR_results.isValid]);
[~, idxATRBestLocal] = min([ATR_results(validATR).ATR]);
ATR_best = ATR_results(validATR(idxATRBestLocal));

%% ============================================================
%% 4) Pareto front with weighted sums
%% ============================================================
DOC_min = DOC_best.FleetDOC;
DOC_max = ATR_best.FleetDOC;

ATR_min = ATR_best.ATR;
ATR_max = DOC_best.ATR;

if abs(DOC_max - DOC_min) < 1e-12
    DOC_max = DOC_min + 1;
end
if abs(ATR_max - ATR_min) < 1e-12
    ATR_max = ATR_min + 1;
end

wList = linspace(0,1,15);   % 1 = DOC only, 0 = ATR only
Pareto = struct([]);
p = 0;

fprintf('\n============================================================\n');
fprintf('BUILDING PARETO FRONT\n');
fprintf('============================================================\n');

for iw = 1:numel(wList)
    w = wList(iw);

    bestScalar = inf;
    bestPoint = [];

    for FS = fleetList
        objW = @(h) objectiveWeighted(FS, h, base, payload_min, payload_max, ...
            w, DOC_min, DOC_max, ATR_min, ATR_max);

        [h_sol, f_sol, exitflag, output] = fmincon( ...
            objW, alt0, [], [], [], [], alt_lb, alt_ub, ...
            @(h) nonlinearConstraints(FS, h, base, payload_min, payload_max), opts);

        R = evaluateDesign(FS, h_sol, base, payload_min, payload_max);

        if R.isValid && isfinite(f_sol) && (f_sol < bestScalar)
            bestScalar = f_sol;

            bestPoint.w          = w;
            bestPoint.FleetSize  = FS;
            bestPoint.Payload    = R.Payload;
            bestPoint.Altitude_m = h_sol;
            bestPoint.FleetDOC   = R.FleetDOC;
            bestPoint.ATR        = R.ATR_scalar;
            bestPoint.Span       = R.Span;
            bestPoint.MTOM       = R.MTOM;
            bestPoint.BlockFuel  = R.BlockFuel;
            bestPoint.exitflag   = exitflag;
            bestPoint.output     = output;
            bestPoint.objective  = f_sol;
        end
    end

    p = p + 1;
    Pareto(p) = bestPoint; %#ok<SAGROW>
end

%% ============================================================
%% 5) Pareto table
%% ============================================================
nP = numel(Pareto);

WeightOnDOC = zeros(nP,1);
FleetSize   = zeros(nP,1);
Payload_kg  = zeros(nP,1);
Altitude_m  = zeros(nP,1);
FleetDOC_MUSDyr = zeros(nP,1);
ATR_value   = zeros(nP,1);
Span_m      = zeros(nP,1);
MTOM_t      = zeros(nP,1);
BlockFuel_t = zeros(nP,1);

for i = 1:nP
    WeightOnDOC(i)   = Pareto(i).w;
    FleetSize(i)     = Pareto(i).FleetSize;
    Payload_kg(i)    = Pareto(i).Payload;
    Altitude_m(i)    = Pareto(i).Altitude_m;
    FleetDOC_MUSDyr(i) = Pareto(i).FleetDOC / 1e6;
    ATR_value(i)     = Pareto(i).ATR;
    Span_m(i)        = Pareto(i).Span;
    MTOM_t(i)        = Pareto(i).MTOM / 1e3;
    BlockFuel_t(i)   = Pareto(i).BlockFuel / 1e3;
end

ParetoTable = table(WeightOnDOC, FleetSize, Payload_kg, Altitude_m, ...
    FleetDOC_MUSDyr, ATR_value, Span_m, MTOM_t, BlockFuel_t);

disp(' ');
disp('Pareto Table:');
disp(ParetoTable);

%% ============================================================
%% 6) Plot Pareto front
%% ============================================================
figure;
plot(FleetDOC_MUSDyr, ATR_value, 'o-', 'LineWidth', 1.5, 'MarkerSize', 6);
xlabel('Fleet DOC [M$/yr]');
ylabel('ATR [-]');
title('Pareto front: Fleet DOC vs ATR');
grid on;
hold on;

plot(DOC_best.FleetDOC/1e6, DOC_best.ATR, 's', 'MarkerSize', 10, 'LineWidth', 1.5);
plot(ATR_best.FleetDOC/1e6, ATR_best.ATR, 'd', 'MarkerSize', 10, 'LineWidth', 1.5);

legend('Pareto points', 'Min DOC', 'Min ATR', 'Location', 'best');

%% ============================================================
%% 7) Pick a balanced compromise point
%% ============================================================
dIdeal = zeros(nP,1);
for i = 1:nP
    DOCn = (Pareto(i).FleetDOC - DOC_min) / (DOC_max - DOC_min);
    ATRn = (Pareto(i).ATR      - ATR_min) / (ATR_max - ATR_min);
    dIdeal(i) = hypot(DOCn, ATRn);
end

[~, idxMid] = min(dIdeal);
BestMid = Pareto(idxMid);

fprintf('\n============================================================\n');
fprintf('BEST DOC SOLUTION\n');
fprintf('============================================================\n');
fprintf('Fleet size                 : %d\n', DOC_best.FleetSize);
fprintf('Payload / aircraft         : %.1f kg\n', DOC_best.Payload);
fprintf('Cruise altitude            : %.1f m\n', DOC_best.Altitude_m);
fprintf('Fleet DOC                  : %.3f M$/yr\n', DOC_best.FleetDOC/1e6);
fprintf('ATR                        : %.6f\n', DOC_best.ATR);
fprintf('Wingspan                   : %.3f m\n', DOC_best.Span);

fprintf('\n============================================================\n');
fprintf('BEST ATR SOLUTION\n');
fprintf('============================================================\n');
fprintf('Fleet size                 : %d\n', ATR_best.FleetSize);
fprintf('Payload / aircraft         : %.1f kg\n', ATR_best.Payload);
fprintf('Cruise altitude            : %.1f m\n', ATR_best.Altitude_m);
fprintf('Fleet DOC                  : %.3f M$/yr\n', ATR_best.FleetDOC/1e6);
fprintf('ATR                        : %.6f\n', ATR_best.ATR);
fprintf('Wingspan                   : %.3f m\n', ATR_best.Span);

fprintf('\n============================================================\n');
fprintf('BEST COMPROMISE SOLUTION\n');
fprintf('============================================================\n');
fprintf('Weight on DOC              : %.3f\n', BestMid.w);
fprintf('Fleet size                 : %d\n', BestMid.FleetSize);
fprintf('Payload / aircraft         : %.1f kg\n', BestMid.Payload);
fprintf('Cruise altitude            : %.1f m\n', BestMid.Altitude_m);
fprintf('Fleet DOC                  : %.3f M$/yr\n', BestMid.FleetDOC/1e6);
fprintf('ATR                        : %.6f\n', BestMid.ATR);
fprintf('Wingspan                   : %.3f m\n', BestMid.Span);

save('PayloadAltitude_IntegerFleet_Pareto.mat', ...
    'DOC_results', 'ATR_results', 'Pareto', 'ParetoTable', ...
    'DOC_best', 'ATR_best', 'BestMid');

function ADP = makeBaselineADP()

ADP = B777.ADP();
ADP.TLAR = cast.TLAR.B777F();

ADP.TLAR.Payload = 91000;
ADP.cruise_altitude = 11800;

ADP.KinkPos = 10;
ADP.TLAR.M_c = 0.85;
ADP.AR = 11;
ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
ADP.WingLoading = 7000;

ADP.Fleet_size = ADP.Total_Payload / ADP.TLAR.Payload;

[ADP.CabinLength, ADP.CabinRadius, ADP.L_total] = ...
    B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ...
                             ADP.CockpitLength, ADP.D_max);

ADP.WingPos = 0.44 * ADP.L_total;
ADP.V_HT    = 0.75;
ADP.V_VT    = 0.07;
ADP.HtpPos  = 0.85 * ADP.L_total;
ADP.VtpPos  = 0.82 * ADP.L_total;

ADP.MTOM    = 3.35 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.19;
ADP.Mf_res  = 0.03;
ADP.Mf_Ldg  = 0.68;
ADP.Mf_TOC  = 0.97;

end

function R = evaluateDesign(FleetSize, cruise_altitude, baseADP, payload_min, payload_max)

BIG = 1e15;

R = struct();
R.isValid = false;
R.FleetDOC = BIG;
R.ATR_scalar = BIG;
R.BlockFuel = BIG;
R.MTOM = BIG;
R.Span = BIG;
R.Payload = NaN;
R.ADP = [];

try
    ADP = baseADP;

    ADP.Fleet_size = FleetSize;
    ADP.TLAR.Payload = ADP.Total_Payload / FleetSize;
    ADP.cruise_altitude = cruise_altitude;
    R.Payload = ADP.TLAR.Payload;

    % Payload feasibility
    if ADP.TLAR.Payload < payload_min || ADP.TLAR.Payload > payload_max
        return
    end

    [ADP.CabinLength, ADP.CabinRadius, ADP.L_total] = ...
        B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ...
                                 ADP.CockpitLength, ADP.D_max);

    ADP.WingPos = 0.44 * ADP.L_total;
    ADP.HtpPos  = 0.85 * ADP.L_total;
    ADP.VtpPos  = 0.82 * ADP.L_total;

    ADP.MTOM = 3.35 * ADP.TLAR.Payload;

    ADP = B777.Size(ADP);
    [~, ~] = B777.BuildGeometry(ADP);

    [BlockFuel, ~, ~, ~, BlockTime_hr, Mission] = ...
        B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

    FuelType = 'JetA1';
    FlightsPerYear      = ADP.TLAR.FlightsPerYear;
    AnnualUtilisationHr = ADP.TLAR.FlightHours;
    NumLandings         = 1;
    NumCycles           = 1;

    HullValue_guess = 44880 * ADP.MTOM^0.65;
    TotalInvestment = HullValue_guess;

    Econ = B777.Economics( ...
        ADP.MTOM, ...
        ADP.Span, ...
        BlockFuel, ...
        BlockTime_hr, ...
        FuelType, ...
        NumLandings, ...
        NumCycles, ...
        AnnualUtilisationHr, ...
        FlightsPerYear, ...
        TotalInvestment, ...
        'ParkingDaysPerYear', ADP.TLAR.ParkingDays, ...
        'NavChargePerFlight', 0, ...
        'UseImprovedMaint', false, ...
        'NumEngines', 2, ...
        'EngineCost', 0);

    cm     = cast.ClimateModel.fromMissionAnalysis(BlockFuel, ADP.TLAR.Range, BlockTime_hr, Mission);
    result = cm.computeImpact();
    atr    = cm.computeATR(result);

    ATR_scalar = extractATRscalar(atr, result);

    FleetDOC = ADP.Fleet_size * Econ.DOC_annual;

    R.isValid    = true;
    R.FleetDOC   = FleetDOC;
    R.ATR_scalar = ATR_scalar;
    R.BlockFuel  = BlockFuel;
    R.MTOM       = ADP.MTOM;
    R.Span       = ADP.Span;
    R.Payload    = ADP.TLAR.Payload;
    R.ADP        = ADP;
    R.Econ       = Econ;
    R.Mission    = Mission;
    R.atr        = atr;
    R.result     = result;

catch ME
    warning('Evaluation failed for FleetSize=%d, Alt=%.1f m: %s', ...
        FleetSize, cruise_altitude, ME.message);
end

end

function ATR_scalar = extractATRscalar(atr, result)

if nargin < 2
    result = [];
end

% Case 1: ATR returned directly as a scalar numeric
if isnumeric(atr) && isscalar(atr) && isfinite(atr)
    ATR_scalar = atr;
    return
end

% Case 2: ATR returned as a struct from ClimateModel.computeATR
if isstruct(atr)
    if isfield(atr,'ATR') && isscalar(atr.ATR) && isfinite(atr.ATR)
        ATR_scalar = atr.ATR;
        return
    elseif isfield(atr,'ATR100') && isscalar(atr.ATR100) && isfinite(atr.ATR100)
        ATR_scalar = atr.ATR100;
        return
    elseif isfield(atr,'total') && isscalar(atr.total) && isfinite(atr.total)
        ATR_scalar = atr.total;
        return
    end
end

% Case 3: fall back to result struct if available
if isstruct(result)
    if isfield(result,'ATR') && isscalar(result.ATR) && isfinite(result.ATR)
        ATR_scalar = result.ATR;
        return
    elseif isfield(result,'ATR100') && isscalar(result.ATR100) && isfinite(result.ATR100)
        ATR_scalar = result.ATR100;
        return
    elseif isfield(result,'TotalImpact') && isscalar(result.TotalImpact) && isfinite(result.TotalImpact)
        ATR_scalar = result.TotalImpact;
        return
    elseif isfield(result,'CO2eq_total') && isscalar(result.CO2eq_total) && isfinite(result.CO2eq_total)
        ATR_scalar = result.CO2eq_total;
        return
    end
end

error('No valid scalar ATR metric found. Update extractATRscalar().');
end
function f = objectiveDOC(FleetSize, cruise_altitude, baseADP, payload_min, payload_max)
    R = evaluateDesign(FleetSize, cruise_altitude, baseADP, payload_min, payload_max);
    f = R.FleetDOC;
end

function f = objectiveATR(FleetSize, cruise_altitude, baseADP, payload_min, payload_max)
    R = evaluateDesign(FleetSize, cruise_altitude, baseADP, payload_min, payload_max);
    f = R.ATR_scalar;
end

function f = objectiveWeighted(FleetSize, cruise_altitude, baseADP, payload_min, payload_max, ...
    wDOC, DOC_min, DOC_max, ATR_min, ATR_max)

    R = evaluateDesign(FleetSize, cruise_altitude, baseADP, payload_min, payload_max);

    DOCn = (R.FleetDOC   - DOC_min) / (DOC_max - DOC_min);
    ATRn = (R.ATR_scalar - ATR_min) / (ATR_max - ATR_min);

    f = wDOC * DOCn + (1 - wDOC) * ATRn;
end

function [c, ceq] = nonlinearConstraints(FleetSize, cruise_altitude, baseADP, payload_min, payload_max)

    BIG = 1e6;

    try
        R = evaluateDesign(FleetSize, cruise_altitude, baseADP, payload_min, payload_max);

        if ~R.isValid
            c = BIG;
            ceq = [];
            return
        end

        % Inequality constraints c <= 0
        c1 = R.Span - 72.0;           % wingspan <= 72 m
        c2 = payload_min - R.Payload; % payload >= payload_min
        c3 = R.Payload - payload_max; % payload <= payload_max

        c = [c1; c2; c3];
        ceq = [];

    catch
        c = BIG;
        ceq = [];
    end
end