%% TradeStudy_AR_FleetSize_DOC
% Trade study on:
%   1) Aspect Ratio
%   2) Fleet Size
%
% Outputs:
%   - summary table in workspace
%   - CSV export
%   - MAT file
%   - plots for both per-aircraft and fleet DOC

clear; clc; close all;

%% ============================================================
%% 1) Baseline definition
%% ============================================================
base = struct();

% ------------------------- Instantiate aircraft -------------------------
base.ADP = B777.ADP();
base.ADP.TLAR = cast.TLAR.B777F();

% ------------------------- Baseline settings ---------------------------
base.ADP.TLAR.M_c = 0.80;
base.ADP.Fleet_size = 10;
base.ADP.TLAR.Payload = base.ADP.Total_Payload / base.ADP.Fleet_size;

base.ADP.AR = 9.8;
base.ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
base.ADP.cruise_altitude = 11800;

% ---------------------- Geometric parameters ---------------------------
base.KinkPos       = 10;
base.WingLoading   = 7000;
base.WingPosFrac   = 0.44;
base.V_HT          = 0.75;
base.V_VT          = 0.07;
base.HtpPosFrac    = 0.85;
base.VtpPosFrac    = 0.82;

% ------------------------ Class-I estimates ----------------------------
base.MTOM_factor   = 3.35;
base.Mf_Fuel       = 0.19;
base.Mf_res        = 0.03;
base.Mf_Ldg        = 0.68;
base.Mf_TOC        = 0.97;

% -------------------------- Cost settings ------------------------------
base.FuelType            = 'JetA1';
base.NumLandings         = 1;
base.NumCycles           = 1;
base.NavChargePerFlight  = 0;
base.UseImprovedMaint    = false;
base.NumEngines          = 2;
base.EngineCost          = 0;

%% ============================================================
%% 2) Wide trade-study ranges
%% ============================================================
AR_values    = [7.0 7.5 8.0 8.5 9.0 9.5 10.0 11.0 12.0 13.0 14.0];
Fleet_values = [4 6 8 10 12 15 18 20];

%% ============================================================
%% 3) Run trade study
%% ============================================================
AllRows = [];

fprintf('\n============================================================\n');
fprintf('      TRADE STUDY: ASPECT RATIO AND FLEET SIZE vs DOC       \n');
fprintf('============================================================\n');

%% ---------------- Aspect ratio sweep ----------------
fprintf('\n--- Sweeping Aspect Ratio ---\n');
for i = 1:numel(AR_values)
    val = AR_values(i);
    fprintf('AR = %.2f ... ', val);

    try
        result = runCase(base, 'AR', val);
        row = makeRow("AR", val, result);
        AllRows = [AllRows; row]; %#ok<AGROW>
        fprintf('OK\n');
    catch ME
        row = makeFailedRow("AR", val, ME.message, base.ADP.Fleet_size);
        AllRows = [AllRows; row]; %#ok<AGROW>
        fprintf('FAILED\n');
    end
end

%% ---------------- Fleet size sweep ----------------
fprintf('\n--- Sweeping Fleet Size ---\n');
for i = 1:numel(Fleet_values)
    val = Fleet_values(i);
    fprintf('Fleet size = %d ... ', val);

    try
        result = runCase(base, 'Fleet_size', val);
        row = makeRow("Fleet_size", val, result);
        AllRows = [AllRows; row]; %#ok<AGROW>
        fprintf('OK\n');
    catch ME
        row = makeFailedRow("Fleet_size", val, ME.message, val);
        AllRows = [AllRows; row]; %#ok<AGROW>
        fprintf('FAILED\n');
    end
end

%% ============================================================
%% 4) Split tables
%% ============================================================
AR_Table    = AllRows(AllRows.Parameter == "AR", :);
Fleet_Table = AllRows(AllRows.Parameter == "Fleet_size", :);

disp(' ');
disp('Full trade study summary:');
disp(AllRows);

disp(' ');
disp('Aspect ratio study:');
disp(AR_Table);

disp(' ');
disp('Fleet size study:');
disp(Fleet_Table);

%% ============================================================
%% 5) Plot results
%% ============================================================

% ---------- Fleet DOC vs AR ----------
T = AR_Table(AR_Table.Success == true, :);
if ~isempty(T)
    figure('Name','Fleet DOC vs Aspect Ratio','Color','w');
    plot(T.Value, T.DOC_annual_MUSD_fleet, '-o', 'LineWidth', 1.5);
    xlabel('Aspect Ratio');
    ylabel('Fleet DOC annual [M$/yr]');
    title('Trade Study: Fleet DOC vs Aspect Ratio');
    grid on;
end

% ---------- Fleet DOC vs Fleet Size ----------
T = Fleet_Table(Fleet_Table.Success == true, :);
if ~isempty(T)
    figure('Name','Fleet DOC vs Fleet Size','Color','w');
    plot(T.Value, T.DOC_annual_MUSD_fleet, '-o', 'LineWidth', 1.5);
    xlabel('Fleet Size');
    ylabel('Fleet DOC annual [M$/yr]');
    title('Trade Study: Fleet DOC vs Fleet Size');
    grid on;
end

% ---------- Per-aircraft DOC vs AR ----------
T = AR_Table(AR_Table.Success == true, :);
if ~isempty(T)
    figure('Name','Per-Aircraft DOC vs Aspect Ratio','Color','w');
    plot(T.Value, T.DOC_annual_MUSD_perAircraft, '-o', 'LineWidth', 1.5);
    xlabel('Aspect Ratio');
    ylabel('Per-aircraft DOC annual [M$/yr]');
    title('Trade Study: Per-Aircraft DOC vs Aspect Ratio');
    grid on;
end

% ---------- Per-aircraft DOC vs Fleet Size ----------
T = Fleet_Table(Fleet_Table.Success == true, :);
if ~isempty(T)
    figure('Name','Per-Aircraft DOC vs Fleet Size','Color','w');
    plot(T.Value, T.DOC_annual_MUSD_perAircraft, '-o', 'LineWidth', 1.5);
    xlabel('Fleet Size');
    ylabel('Per-aircraft DOC annual [M$/yr]');
    title('Trade Study: Per-Aircraft DOC vs Fleet Size');
    grid on;
end

%% ============================================================
%% 6) Best results
%% ============================================================
goodRows = AllRows(AllRows.Success == true, :);

if ~isempty(goodRows)
    goodRowsFleet = sortrows(goodRows, 'DOC_annual_MUSD_fleet', 'ascend');
    goodRowsAircraft = sortrows(goodRows, 'DOC_annual_MUSD_perAircraft', 'ascend');

    fprintf('\n============================================================\n');
    fprintf('              BEST CASES BY LOWEST FLEET DOC                \n');
    fprintf('============================================================\n');
    disp(goodRowsFleet(:, { ...
        'Parameter','Value', ...
        'DOC_annual_MUSD_fleet', ...
        'DOC_annual_MUSD_perAircraft', ...
        'DOC_trip_kUSD', ...
        'MTOM_t','BlockFuel_t','BlockTime_hr','Span_m','Payload_t','Fleet_size'}));

    fprintf('\n============================================================\n');
    fprintf('          BEST CASES BY LOWEST PER-AIRCRAFT DOC             \n');
    fprintf('============================================================\n');
    disp(goodRowsAircraft(:, { ...
        'Parameter','Value', ...
        'DOC_annual_MUSD_perAircraft', ...
        'DOC_annual_MUSD_fleet', ...
        'DOC_trip_kUSD', ...
        'MTOM_t','BlockFuel_t','BlockTime_hr','Span_m','Payload_t','Fleet_size'}));
end

%% ============================================================
%% 7) Save outputs
%% ============================================================
save('TradeStudy_AR_FleetSize_DOC_results.mat', ...
    'base', 'AR_values', 'Fleet_values', 'AllRows', 'AR_Table', 'Fleet_Table');

writetable(AllRows, 'TradeStudy_AR_FleetSize_DOC_summary.csv');

fprintf('\nSaved:\n');
fprintf('  - TradeStudy_AR_FleetSize_DOC_results.mat\n');
fprintf('  - TradeStudy_AR_FleetSize_DOC_summary.csv\n');

%% ============================================================
%% Local functions
%% ============================================================

function row = makeRow(paramName, value, result)
row = table( ...
    string(paramName), ...
    value, ...
    result.Success, ...
    result.DOC_annual_MUSD_perAircraft, ...
    result.DOC_annual_MUSD_fleet, ...
    result.COC_annual_MUSD_perAircraft, ...
    result.COC_annual_MUSD_fleet, ...
    result.DOC_trip_kUSD, ...
    result.COC_trip_kUSD, ...
    result.MTOM_t, ...
    result.BlockFuel_t, ...
    result.BlockTime_hr, ...
    result.Span_m, ...
    result.Payload_t, ...
    result.Fleet_size, ...
    string(result.FailReason), ...
    'VariableNames', { ...
        'Parameter', 'Value', 'Success', ...
        'DOC_annual_MUSD_perAircraft', ...
        'DOC_annual_MUSD_fleet', ...
        'COC_annual_MUSD_perAircraft', ...
        'COC_annual_MUSD_fleet', ...
        'DOC_trip_kUSD', ...
        'COC_trip_kUSD', ...
        'MTOM_t', 'BlockFuel_t', 'BlockTime_hr', ...
        'Span_m', 'Payload_t', 'Fleet_size', 'FailReason'});
end

function row = makeFailedRow(paramName, value, failMsg, fleetSize)
row = table( ...
    string(paramName), ...
    value, ...
    false, ...
    NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, fleetSize, ...
    string(failMsg), ...
    'VariableNames', { ...
        'Parameter', 'Value', 'Success', ...
        'DOC_annual_MUSD_perAircraft', ...
        'DOC_annual_MUSD_fleet', ...
        'COC_annual_MUSD_perAircraft', ...
        'COC_annual_MUSD_fleet', ...
        'DOC_trip_kUSD', ...
        'COC_trip_kUSD', ...
        'MTOM_t', 'BlockFuel_t', 'BlockTime_hr', ...
        'Span_m', 'Payload_t', 'Fleet_size', 'FailReason'});
end

function result = runCase(base, paramName, paramValue)

ADP = base.ADP;

% Keep payload linked to fleet size
ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;

% Apply study variable
switch paramName
    case 'AR'
        ADP.AR = paramValue;

    case 'Fleet_size'
        ADP.Fleet_size = paramValue;
        ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;

    otherwise
        error('Unknown study parameter: %s', paramName);
end

% Fuselage sizing
[ADP.CabinLength, ADP.CabinRadius, ADP.L_total] = ...
    B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ...
                             ADP.CockpitLength, ADP.D_max);

% Geometry positions
ADP.KinkPos     = base.KinkPos;
ADP.WingLoading = base.WingLoading;
ADP.WingPos     = base.WingPosFrac * ADP.L_total;
ADP.V_HT        = base.V_HT;
ADP.V_VT        = base.V_VT;
ADP.HtpPos      = base.HtpPosFrac * ADP.L_total;
ADP.VtpPos      = base.VtpPosFrac * ADP.L_total;

% Class-I estimates
ADP.MTOM    = base.MTOM_factor * ADP.TLAR.Payload;
ADP.Mf_Fuel = base.Mf_Fuel;
ADP.Mf_res  = base.Mf_res;
ADP.Mf_Ldg  = base.Mf_Ldg;
ADP.Mf_TOC  = base.Mf_TOC;

% Sizing
ADP = B777.Size(ADP);

% Build geometry
[B7Geom, B7Mass] = B777.BuildGeometry(ADP); %#ok<NASGU>
d = B7Mass.GetData; %#ok<NASGU>

% Mission analysis
[BlockFuel, ~, ~, ~, BlockTime_hr] = ...
    B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

% Economics
FlightsPerYear      = ADP.TLAR.FlightsPerYear;
AnnualUtilisationHr = ADP.TLAR.FlightHours;

HullValue_guess = 44880 * ADP.MTOM^0.65;
TotalInvestment = HullValue_guess;

Econ = B777.Economics( ...
    ADP.MTOM, ...
    ADP.Span, ...
    BlockFuel, ...
    BlockTime_hr, ...
    base.FuelType, ...
    base.NumLandings, ...
    base.NumCycles, ...
    AnnualUtilisationHr, ...
    FlightsPerYear, ...
    TotalInvestment, ...
    'ParkingDaysPerYear', ADP.TLAR.ParkingDays, ...
    'NavChargePerFlight', base.NavChargePerFlight, ...
    'UseImprovedMaint', base.UseImprovedMaint, ...
    'NumEngines', base.NumEngines, ...
    'EngineCost', base.EngineCost);

% Fleet totals
Fleet_DOC_annual = ADP.Fleet_size * Econ.DOC_annual;
Fleet_COC_annual = ADP.Fleet_size * Econ.COC_annual;

% Return results
result = struct();
result.Success                    = true;
result.FailReason                 = "";

result.DOC_annual_MUSD_perAircraft = Econ.DOC_annual / 1e6;
result.DOC_annual_MUSD_fleet       = Fleet_DOC_annual / 1e6;

result.COC_annual_MUSD_perAircraft = Econ.COC_annual / 1e6;
result.COC_annual_MUSD_fleet       = Fleet_COC_annual / 1e6;

result.DOC_trip_kUSD              = Econ.DOC_trip / 1e3;
result.COC_trip_kUSD              = Econ.COC_trip / 1e3;

result.MTOM_t                     = ADP.MTOM / 1e3;
result.BlockFuel_t                = BlockFuel / 1e3;
result.BlockTime_hr               = BlockTime_hr;
result.Span_m                     = ADP.Span;
result.Payload_t                  = ADP.TLAR.Payload / 1e3;
result.Fleet_size                 = ADP.Fleet_size;
end