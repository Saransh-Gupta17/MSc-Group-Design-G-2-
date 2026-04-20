%% TradeStudy_CruiseMach_BlockFuel
% Trade study:
%   Cruise Mach vs Block Fuel

clear; clc; close all;

%% ============================================================
%% 1) Baseline definition
%% ============================================================
base = struct();

base.ADP = B777.ADP();
base.ADP.TLAR = cast.TLAR.B777F();

% Baseline settings
base.ADP.TLAR.M_c = 0.80;
base.ADP.Fleet_size = 10;
base.ADP.TLAR.Payload = base.ADP.Total_Payload / base.ADP.Fleet_size;

base.ADP.AR = 9.8;
base.ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
base.ADP.cruise_altitude = 11800;

% Geometry parameters
base.KinkPos       = 10;
base.WingLoading   = 7000;
base.WingPosFrac   = 0.44;
base.V_HT          = 0.75;
base.V_VT          = 0.07;
base.HtpPosFrac    = 0.85;
base.VtpPosFrac    = 0.82;

% Class-I estimates
base.MTOM_factor   = 3.35;
base.Mf_Fuel       = 0.19;
base.Mf_res        = 0.03;
base.Mf_Ldg        = 0.68;
base.Mf_TOC        = 0.97;

%% ============================================================
%% 2) Cruise Mach sweep
%% ============================================================
Mach_values = [0.76 0.78 0.80 0.82 0.84 0.86 0.88];

AllRows = [];

fprintf('\n============================================================\n');
fprintf('            TRADE STUDY: CRUISE MACH vs BLOCK FUEL          \n');
fprintf('============================================================\n');

for i = 1:numel(Mach_values)
    val = Mach_values(i);
    fprintf('Cruise Mach = %.3f ... ', val);

    try
        result = runMachCase(base, val);
        row = makeMachRow(val, result);
        AllRows = [AllRows; row]; %#ok<AGROW>
        fprintf('OK\n');
    catch ME
        row = makeFailedMachRow(val, ME.message);
        AllRows = [AllRows; row]; %#ok<AGROW>
        fprintf('FAILED\n');
    end
end

disp(' ');
disp('Cruise Mach study summary:');
disp(AllRows);

%% ============================================================
%% 3) Plot Block Fuel vs Cruise Mach
%% ============================================================
T = AllRows(AllRows.Success == true, :);

if ~isempty(T)
    figure('Name','Block Fuel vs Cruise Mach','Color','w');
    plot(T.CruiseMach, T.BlockFuel_t, '-o', 'LineWidth', 1.5);
    xlabel('Cruise Mach');
    ylabel('Block Fuel [t]');
    title('Trade Study: Cruise Mach vs Block Fuel');
    grid on;
end

%% ============================================================
%% 4) Show best case
%% ============================================================
if ~isempty(T)
    Tsort = sortrows(T, 'BlockFuel_t', 'ascend');

    fprintf('\n============================================================\n');
    fprintf('              BEST CASES BY LOWEST BLOCK FUEL               \n');
    fprintf('============================================================\n');
    disp(Tsort(:, { ...
        'CruiseMach', ...
        'BlockFuel_t', ...
        'TripFuel_t', ...
        'ResFuel_t', ...
        'MTOM_t', ...
        'BlockTime_hr', ...
        'Span_m', ...
        'Payload_t'}));
end

%% ============================================================
%% 5) Save outputs
%% ============================================================
save('TradeStudy_CruiseMach_BlockFuel_results.mat', ...
    'base', 'Mach_values', 'AllRows');

writetable(AllRows, 'TradeStudy_CruiseMach_BlockFuel_summary.csv');

fprintf('\nSaved:\n');
fprintf('  - TradeStudy_CruiseMach_BlockFuel_results.mat\n');
fprintf('  - TradeStudy_CruiseMach_BlockFuel_summary.csv\n');

%% ============================================================
%% Local functions
%% ============================================================

function row = makeMachRow(machVal, result)
row = table( ...
    machVal, ...
    result.Success, ...
    result.BlockFuel_t, ...
    result.TripFuel_t, ...
    result.ResFuel_t, ...
    result.MTOM_t, ...
    result.BlockTime_hr, ...
    result.Span_m, ...
    result.Payload_t, ...
    string(result.FailReason), ...
    'VariableNames', { ...
        'CruiseMach', 'Success', ...
        'BlockFuel_t', 'TripFuel_t', 'ResFuel_t', ...
        'MTOM_t', 'BlockTime_hr', 'Span_m', 'Payload_t', 'FailReason'});
end

function row = makeFailedMachRow(machVal, failMsg)
row = table( ...
    machVal, ...
    false, ...
    NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
    string(failMsg), ...
    'VariableNames', { ...
        'CruiseMach', 'Success', ...
        'BlockFuel_t', 'TripFuel_t', 'ResFuel_t', ...
        'MTOM_t', 'BlockTime_hr', 'Span_m', 'Payload_t', 'FailReason'});
end

function result = runMachCase(base, machVal)

ADP = base.ADP;

% Apply cruise Mach
ADP.TLAR.M_c = machVal;

% Keep payload linked to fleet size
ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;

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

% Full sizing
ADP = B777.Size(ADP);

% Mission analysis
[BlockFuel, TripFuel, ResFuel, ~, MissionTime] = ...
    B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

% Return results
result = struct();
result.Success      = true;
result.FailReason   = "";

result.BlockFuel_t  = BlockFuel / 1e3;
result.TripFuel_t   = TripFuel / 1e3;
result.ResFuel_t    = ResFuel / 1e3;
result.MTOM_t       = ADP.MTOM / 1e3;
result.BlockTime_hr = MissionTime / 3600;
result.Span_m       = ADP.Span;
result.Payload_t    = ADP.TLAR.Payload / 1e3;
end