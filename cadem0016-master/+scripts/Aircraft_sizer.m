%% ExampleSizing_Baseline
% Sizes a single aircraft configuration, plots it, and prints a clean
% summary of geometry, mass, aero and economics for the selected fleet case.
clear; clc; close all;
clear; clc; close all;

% Store current graphics default
oldFigVisible = get(0,'DefaultFigureVisible');

% Hide all figures created anywhere during optimisation
set(0,'DefaultFigureVisible','off');

cleanupObj = onCleanup(@() set(0,'DefaultFigureVisible', oldFigVisible));

%% ------------------------- Instantiate aircraft -------------------------
ADP = B777.ADP();
ADP.TLAR = cast.TLAR.B777F();

%% ------------------------- Hyper-parameters ----------------------------

ADP.TLAR.Payload = 91000;
ADP.cruise_altitude = 11800;

%% ---------------------- Geometric parameters ---------------------------
ADP.KinkPos = 10;   % spanwise position of TE kink in wing planform
ADP.TLAR.M_c = 0.85;
ADP.AR = 11;
ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
ADP.Fleet_size = ADP.Total_Payload/ADP.TLAR.Payload; %NOT A HYPERPARAMTEER
ADP.WingLoading = 7000;
ADP.TLAR.M_c = 0.84;
ADP.Fleet_size = 10;
ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;

ADP.AR = 8.5;
ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
ADP.WingLoading = 9000;
ADP.cruise_altitude = 12000;

%% ---------------------- Geometric parameters ---------------------------
ADP.KinkPos = 10;

[ADP.CabinLength, ADP.CabinRadius, ADP.L_total] = ...
    B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ...
                             ADP.CockpitLength, ADP.D_max);

ADP.WingPos = 0.50 * ADP.L_total;
ADP.V_HT    = 1.3;
ADP.V_VT    = 0.07;
ADP.HtpPos  = 0.88 * ADP.L_total;
ADP.VtpPos  = 0.88 * ADP.L_total;

%% ------------------------ Class-I estimates ----------------------------
ADP.MTOM    = 3.35 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.19;
ADP.Mf_res  = 0.03;
ADP.Mf_Ldg  = 0.68;
ADP.Mf_TOC  = 0.97;

%% ------------------------------ Sizing ---------------------------------
ADP = B777.Size(ADP);

%% ------------------- Build geometry and masses -------------------------
[B7Geom, B7Mass] = B777.BuildGeometry(ADP);
d = B7Mass.GetData;

%% =========================
% EMPENNAGE
%% =========================
[GeomEmp, MassEmp] = B777.geom.empenage(ADP);

B7Geom = [B7Geom GeomEmp];
B7Mass = [B7Mass MassEmp];
%% ------------------------ STABILITY ANALYSIS --------------------------
cfg = struct();

% Aerodynamics
cfg.Cm0 = -0.05;
cfg.eta_h = 0.9;
cfg.CLh_max_elevator = -0.8;

% Stability target
cfg.StaticMargin_target = 0.08;

% CG envelope assumptions
cfg.PayloadFwdFrac = 0.2;
cfg.PayloadAftFrac = 0.8;
cfg.PayloadFracs   = [0.6 1.0];
cfg.FuelFracs      = [0.3 0.6 1.0];

% Atmosphere
cfg.rho_cruise = 0.38;
cfg.a_cruise   = 295;

% Engine & control
cfg.T_engine_TO = ADP.Thrust / 2;
cfg.y_engine    = 0.25 * ADP.Span;

% Speeds
cfg.V_TO    = 80;
cfg.V_cross = 20;

% Side force model
cfg.SideAreaFuse = 200;
cfg.CY_cross     = 0.5;
cfg.xcp_cross    = ADP.WingPos;
cfg.CY_v_max_rud = 1.2;

% Geometry from ADP
cfg.Sref   = ADP.WingArea;
cfg.Span   = ADP.Span;
cfg.MAC    = ADP.c_ac;
cfg.AR     = ADP.AR;

cfg.WingPos = ADP.WingPos;
cfg.HtpPos  = ADP.HtpPos;
cfg.VtpPos  = ADP.VtpPos;

cfg.VH = ADP.V_HT;
cfg.VV = ADP.V_VT;

cfg.CabinLength   = ADP.CabinLength;
cfg.CockpitLength = ADP.CockpitLength;

% Neutral point reference
cfg.xLEMAC = ADP.WingPos - 0.25 * ADP.c_ac;

%% ------------------ RUN STABILITY ------------------
[AircraftStable, Summary, Weightbook, Results] = ...
    B777.Stability(cfg, ADP, B7Geom, B7Mass);

disp(Results)
%% ------------------------- Mission analysis ----------------------------
[BlockFuel, ~, ~, ~, ~] = B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

%% -------------------------- Cost analysis ------------------------------
FuelType = 'JetA1';

[CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, ...
    MaintFixed, MaintVar, InsuranceCost, TotalCost] = ...
    B777.Economics(ADP.MTOM, ADP.Span, BlockFuel, ...
                   ADP.TLAR.FlightHours, ADP.TLAR.ParkingDays, FuelType);

FlightsPerYear = ADP.TLAR.FlightsPerYear;

FleetCrewCost      = ADP.Fleet_size * CrewCost;
FleetLandingCost   = ADP.Fleet_size * LandingFee * FlightsPerYear;
FleetParkingCost   = ADP.Fleet_size * ParkingFee;
FleetFuelCost      = ADP.Fleet_size * FuelCost * FlightsPerYear;
FleetHullValue     = ADP.Fleet_size * HullValue;
FleetMaintFixed    = ADP.Fleet_size * MaintFixed;
FleetMaintVar      = ADP.Fleet_size * MaintVar;
FleetInsuranceCost = ADP.Fleet_size * InsuranceCost;

FleetTotalDOC = FleetCrewCost + FleetLandingCost + FleetParkingCost + ...
                FleetFuelCost + FleetMaintFixed + FleetMaintVar + ...
                FleetInsuranceCost;

%% ----------------------------- Plotting --------------------------------
f = figure(1);
clf;

img = imread('B777F_planform.png');
imshow(img, 'XData', [0 63.7], 'YData', [-64.8 64.8]/2);
hold on

cast.draw(B7Geom, B7Mass)

ax = gca;
ax.XAxis.Visible = "on";
ax.YAxis.Visible = "on";
axis equal
ylim([-0.5 0.5] * ADP.Span)
title('Sized Aircraft Geometry')

%% ------------------------- Mission analysis ----------------------------
[BlockFuel, ~, ~, ~, BlockTime_hr,Mission] = B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

%% -------------------------- Cost analysis ------------------------------
FuelType = 'JetA1';

FlightsPerYear      = ADP.TLAR.FlightsPerYear;
AnnualUtilisationHr = ADP.TLAR.FlightHours;
NumLandings         = 1;
NumCycles           = 1;

% Total investment placeholder:
% for now use hull value as a proxy until you build the initial cost model
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

% Per-aircraft extracted values
HullValue      = Econ.breakdown.HullValue;
FuelCost       = Econ.breakdown.FuelTrip;
LandingFee     = Econ.breakdown.LandingFeeTrip;
ParkingFee     = Econ.breakdown.ParkingTrip * FlightsPerYear;
InsuranceCost  = Econ.breakdown.InsuranceAnnual;
MaintFixed     = Econ.breakdown.MaintFixedAnnual;
MaintVar       = Econ.breakdown.MaintVarAnnual;

CrewCost = (Econ.breakdown.CockpitCrewTrip + Econ.breakdown.CabinCrewTrip) ...
           * FlightsPerYear;

TotalCost = Econ.DOC_annual;

% Fleet annual values
FleetCrewCost      = ADP.Fleet_size * CrewCost;
FleetLandingCost   = ADP.Fleet_size * LandingFee * FlightsPerYear;
FleetParkingCost   = ADP.Fleet_size * ParkingFee;
FleetFuelCost      = ADP.Fleet_size * FuelCost * FlightsPerYear;
FleetHullValue     = ADP.Fleet_size * HullValue;
FleetMaintFixed    = ADP.Fleet_size * MaintFixed;
FleetMaintVar      = ADP.Fleet_size * MaintVar;
FleetInsuranceCost = ADP.Fleet_size * InsuranceCost;

FleetTotalDOC = ADP.Fleet_size * Econ.DOC_annual;
FleetTotalCOC = ADP.Fleet_size * Econ.COC_annual;

%% Climate Impact model
cm     = cast.ClimateModel.fromMissionAnalysis(BlockFuel, ADP.TLAR.Range, BlockTime_hr,Mission);
result = cm.computeImpact();
atr    = cm.computeATR(result);
%cm.plotEmissions(result);

%
%% ----------------------------- Plotting --------------------------------
f = figure(1);
clf;

img = imread('B777F_planform.png');
imshow(img, 'XData', [0 63.7], 'YData', [-64.8 64.8]/2);
hold on

cast.draw(B7Geom, B7Mass)

ax = gca;
ax.XAxis.Visible = "on";
ax.YAxis.Visible = "on";
axis equal
ylim([-0.5 0.5] * ADP.Span)
title('Sized Aircraft Geometry')

%% -------------------------- Printed summary ----------------------------
fprintf('\n============================================================\n');
fprintf('                 SINGLE AIRCRAFT SIZING SUMMARY             \n');
fprintf('============================================================\n');

fprintf('\n--- Fleet / Mission Inputs ---------------------------------\n');
fprintf('Fleet size                         : %d aircraft\n', ADP.Fleet_size);
fprintf('Payload per aircraft               : %8.1f t\n', ADP.TLAR.Payload/1e3);
fprintf('Range                              : %8.1f km\n', ADP.TLAR.Range/1e3);
fprintf('Cruise Mach                        : %8.3f\n', ADP.TLAR.M_c);
fprintf('Aspect ratio                       : %8.3f\n', ADP.AR);

fprintf('\n--- Main Geometry ------------------------------------------\n');
fprintf('Fuselage total length              : %8.2f m\n', ADP.L_total);
fprintf('Cabin length                       : %8.2f m\n', ADP.CabinLength);
fprintf('Cabin radius                       : %8.2f m\n', ADP.CabinRadius);
fprintf('Wing span                          : %8.2f m\n', ADP.Span);
fprintf('Wing position                      : %8.2f m\n', ADP.WingPos);
fprintf('HTP position                       : %8.2f m\n', ADP.HtpPos);
fprintf('VTP position                       : %8.2f m\n', ADP.VtpPos);
fprintf('\n--- Mass / Fuel --------------------------------------------\n');
fprintf('MTOM                               : %8.1f t\n', ADP.MTOM/1e3);
fprintf('Estimated fuel mass                : %8.1f t\n', ADP.Mf_Fuel*ADP.MTOM/1e3);
fprintf('Block fuel per mission             : %8.1f t\n', BlockFuel/1e3);

% --- see all wing entries (please dont remove my code keeps breaking) ---
wingIdxAll = contains(d(:,1), "Wing");

disp("---- ALL WING ENTRIES ----")
disp(d(wingIdxAll,:))

% --- pick correct one ---
wingIdx = strcmp(d(:,1), "Wing_Structural");

if any(wingIdx)
    wing_mass = str2double(d{wingIdx,2});
    fprintf('Wing mass                          : %8.1f t\n', wing_mass);
else
    warning("Wing_Structural not found!")
end

fprintf('\n--- Aerodynamics -------------------------------------------\n');
fprintf('CD0                                : %8.4f\n', ADP.AeroPolar.CD(0));
fprintf('CD at CL = 0.5                     : %8.4f\n', ADP.AeroPolar.CD(0.5));

fprintf('\n--- Economics: Per Aircraft --------------------------------\n');
fprintf('COC                                : %8.3f M$/yr\n', Econ.COC_annual/1e6);
fprintf('DOC                                : %8.3f M$/yr\n', Econ.DOC_annual/1e6);
fprintf('COC                                : %8.3f k$/flight\n', Econ.COC_trip/1e3);
fprintf('DOC                                : %8.3f k$/flight\n', Econ.DOC_trip/1e3);
fprintf('Crew cost                          : %8.3f M$/yr\n', CrewCost/1e6);
fprintf('Landing fee                        : %8.3f k$/flight\n', LandingFee/1e3);
fprintf('Parking fee                        : %8.3f M$/yr\n', ParkingFee/1e6);
fprintf('Fuel cost                          : %8.3f k$/flight\n', FuelCost/1e3);
fprintf('Hull value                         : %8.3f M$\n', HullValue/1e6);
fprintf('Fixed maintenance                  : %8.3f M$/yr\n', MaintFixed/1e6);
fprintf('Variable maintenance               : %8.3f M$/yr\n', MaintVar/1e6);
fprintf('Insurance cost                     : %8.3f M$/yr\n', InsuranceCost/1e6);

fprintf('\n--- Economics: Entire Fleet --------------------------------\n');
fprintf('Fleet COC                          : %8.3f M$/yr\n', FleetTotalCOC/1e6);
fprintf('Fleet DOC                          : %8.3f M$/yr\n', FleetTotalDOC/1e6);
fprintf('Fleet crew cost                    : %8.3f M$/yr\n', FleetCrewCost/1e6);
fprintf('Fleet landing fees                 : %8.3f M$/yr\n', FleetLandingCost/1e6);
fprintf('Fleet parking fees                 : %8.3f M$/yr\n', FleetParkingCost/1e6);
fprintf('Fleet fuel cost                    : %8.3f M$/yr\n', FleetFuelCost/1e6);
fprintf('Fleet hull value                   : %8.3f M$\n', FleetHullValue/1e6);
fprintf('Fleet fixed maintenance            : %8.3f M$/yr\n', FleetMaintFixed/1e6);
fprintf('Fleet variable maintenance         : %8.3f M$/yr\n', FleetMaintVar/1e6);
fprintf('Fleet insurance cost               : %8.3f M$/yr\n', FleetInsuranceCost/1e6);

%% ---------------------- Optional summary tables ------------------------
CostNames = { ...
    'Crew Cost / aircraft / year'
    'Landing Fee / aircraft / flight'
    'Parking Fee / aircraft / year'
    'Fuel Cost / aircraft / flight'
    'Hull Value / aircraft'
    'Maint. Fixed / aircraft / year'
    'Maint. Variable / aircraft / year'
    'Insurance / aircraft / year'
    'Fleet Crew Cost / year'
    'Fleet Landing Fees / year'
    'Fleet Parking Fees / year'
    'Fleet Fuel Cost / year'
    'Fleet Hull Value'
    'Fleet Maint. Fixed / year'
    'Fleet Maint. Variable / year'
    'Fleet Insurance / year'
    'Fleet Total DOC / year'};

CostValues_MUSD = [ ...
    CrewCost
    LandingFee
    ParkingFee
    FuelCost
    HullValue
    MaintFixed
    MaintVar
    InsuranceCost
    FleetCrewCost
    FleetLandingCost
    FleetParkingCost
    FleetFuelCost
    FleetHullValue
    FleetMaintFixed
    FleetMaintVar
    FleetInsuranceCost
    FleetTotalDOC] / 1e6;

CostSummary = table(CostNames, CostValues_MUSD, ...
    'VariableNames', {'Cost_Item', 'Value_MUSD'});

disp(' ');
disp('Cost Summary Table:');
disp(CostSummary);

fprintf('\n--- Engine Properties --------------------------------------\n');

eng = ADP.Engine;

props = properties(eng);

for i = 1:length(props)
    val = eng.(props{i});

    if isnumeric(val) && isscalar(val)
        fprintf('%-30s : %g\n', props{i}, val);

    elseif isnumeric(val) && ~isscalar(val)
        fprintf('%-30s : [%s]\n', props{i}, num2str(val));

    elseif ischar(val) || isstring(val)
        fprintf('%-30s : %s\n', props{i}, val);

    else
        fprintf('%-30s : [non-displayable type]\n', props{i});
    end
end

[BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, Mission, CriticalTW, CriticalWS] = ...
    B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

%% ============================================================
%% 21) Detailed mission printout
%% ============================================================
printMissionReport(Mission, ADP.MTOM);

%% ============================================================
%% 22) Altitude history plot
%% ============================================================
plotMissionHistory(Mission, ADP.MTOM);

function printMissionReport(Mission, M_TO)

fprintf('\n========================================================================================================\n');
fprintf('                                      DETAILED MISSION REPORT                                           \n');
fprintf('========================================================================================================\n');
fprintf('%-20s %9s %11s %11s %11s %11s %12s %12s %11s %8s\n', ...
    'Segment', 'Time(min)', 'h_start(m)', 'h_end(m)', ...
    'Range(km)', 'Fuel(kg)', 'MassEnd(kg)', 'Speed/Mach', 'Thrust(kN)', 'C_L');
fprintf('%s\n', repmat('-',1,135));

m_current = M_TO;

    function printRow(name, seg, h1, h2, speedStr, Thrust_kN, CL)
        if nargin < 3 || isempty(h1), h1 = NaN; end
        if nargin < 4 || isempty(h2), h2 = NaN; end
        if nargin < 5 || isempty(speedStr), speedStr = '-'; end
        if nargin < 6 || isempty(Thrust_kN), Thrust_kN = NaN; end
        if nargin < 7 || isempty(CL), CL = NaN; end

        fuel  = getFieldOr(seg, 'Fuel', 0);
        time  = getFieldOr(seg, 'Time', 0);
        range = getFieldOr(seg, 'Range', 0);

        m_current_local = m_current - fuel;

        fprintf('%-20s %9.2f %11.1f %11.1f %11.1f %11.1f %12.1f %12s %11.1f %8.3f\n', ...
            name, time/60, h1, h2, range/1000, fuel, m_current_local, speedStr, Thrust_kN, CL);

        m_current = m_current_local;
    end

% ---------------- Main mission ----------------
printRow('TaxiOut', Mission.TaxiOut, 0, 0, 'ground', ...
    getFieldOr(Mission.TaxiOut,'Treq',NaN)/1000, NaN);

printRow('TakeoffGround', Mission.TakeoffGround, 0, 0, ...
    sprintf('Vlof %.1f', getFieldOr(Mission.TakeoffGround,'Vlof',NaN)), ...
    getFieldOr(Mission.TakeoffGround,'Treq',NaN)/1000, ...
    getFieldOr(Mission.TakeoffGround,'CL',NaN));

printRow('InitialClimb', Mission.InitialClimb, ...
    getFieldOr(Mission.InitialClimb,'h_start',0), ...
    getFieldOr(Mission.InitialClimb,'h_end',NaN), ...
    sprintf('%.1f m/s', getFieldOr(Mission.InitialClimb,'V',NaN)), ...
    getFieldOr(Mission.InitialClimb,'Treq',NaN)/1000, ...
    getFieldOr(Mission.InitialClimb,'CL',NaN));

printVerticalSegmentRow('Climb1', Mission.Climb1);
printVerticalSegmentRow('Climb2', Mission.Climb2);
printVerticalSegmentRow('Climb3', Mission.Climb3);

printCruiseSegmentRow('Cruise', Mission.Cruise);

printVerticalSegmentRow('Descent1', Mission.Descent1);
printVerticalSegmentRow('Descent2', Mission.Descent2);
printVerticalSegmentRow('Descent3', Mission.Descent3);

printRow('ApproachLanding', Mission.ApproachLanding, 1500*0.3048, 0, ...
    sprintf('%.1f m/s', getFieldOr(Mission.ApproachLanding,'V',NaN)), ...
    getFieldOr(Mission.ApproachLanding,'Treq',NaN)/1000, ...
    getFieldOr(Mission.ApproachLanding,'CL',NaN));

% ---------------- Reserve mission ----------------
printVerticalSegmentRow('AltClimb', Mission.Alternate.Climb);
printCruiseSegmentRow('AltCruise', Mission.Alternate.Cruise);
printVerticalSegmentRow('AltDescent', Mission.Alternate.Descent);

printRow('AltApproach', Mission.Alternate.Approach, 0, 0, 'M 0.20', ...
    getFieldOr(Mission.Alternate.Approach,'Treq',NaN)/1000, ...
    getFieldOr(Mission.Alternate.Approach,'CL',NaN));

printRow('Loiter', Mission.Loiter, 1500*0.3048, 1500*0.3048, ...
    sprintf('M %.2f', estimateMachFromLoiter(Mission.Loiter)), ...
    getFieldOr(Mission.Loiter,'Treq',NaN)/1000, ...
    getFieldOr(Mission.Loiter,'CL',NaN));

printRow('Contingency', Mission.Contingency, 1500*0.3048, 1500*0.3048, '-', ...
    getFieldOr(Mission.Contingency,'Treq',NaN)/1000, ...
    getFieldOr(Mission.Contingency,'CL',NaN));

fprintf('%s\n', repmat('-',1,135));
fprintf('Trip fuel                 = %.1f kg\n', Mission.Total.TripFuel);
fprintf('Reserve fuel              = %.1f kg\n', Mission.Total.ResFuel);
fprintf('Block fuel                = %.1f kg\n', Mission.Total.BlockFuel);

if isfield(Mission.Total,'AirborneDesignMissionTime')
    fprintf('Airborne design time      = %.2f hr\n', Mission.Total.AirborneDesignMissionTime/3600);
end
if isfield(Mission.Total,'DesignMissionTime')
    fprintf('Design mission time       = %.2f hr\n', Mission.Total.DesignMissionTime/3600);
end
if isfield(Mission.Total,'TotalMissionTime')
    fprintf('Total mission time        = %.2f hr\n', Mission.Total.TotalMissionTime/3600);
elseif isfield(Mission.Total,'Time')
    fprintf('Total mission time        = %.2f hr\n', Mission.Total.Time/3600);
end

fprintf('========================================================================================================\n'); 

    function printVerticalSegmentRow(name, seg)
        h1 = NaN;
        h2 = NaN;
        speedStr = '-';

        if isfield(seg,'StepAlt') && ~isempty(seg.StepAlt)
            if numel(seg.StepAlt) == 1
                h1 = seg.StepAlt(1);
                h2 = seg.StepAlt(1);
            else
                dh = mean(diff(seg.StepAlt), 'omitnan');
                h1 = seg.StepAlt(1) - abs(dh)/2;
                h2 = seg.StepAlt(end) + abs(dh)/2;

                if contains(name,'Descent')
                    tmp = h1;
                    h1 = h2;
                    h2 = tmp;
                end
            end
        end

        if isfield(seg,'StepV') && ~isempty(seg.StepV)
            speedStr = sprintf('%.1f m/s', mean(seg.StepV,'omitnan'));
        elseif isfield(seg,'StepMach') && ~isempty(seg.StepMach)
            speedStr = sprintf('M %.2f', mean(seg.StepMach,'omitnan'));
        end

        Thrust_seg = NaN;
        CL_seg = NaN;

        if isfield(seg,'StepTreq') && ~isempty(seg.StepTreq)
            Thrust_seg = mean(seg.StepTreq,'omitnan') / 1000;
        elseif isfield(seg,'Treq')
            Thrust_seg = seg.Treq / 1000;
        end

        if isfield(seg,'StepCL') && ~isempty(seg.StepCL)
            CL_seg = mean(seg.StepCL,'omitnan');
        elseif isfield(seg,'CL')
            CL_seg = seg.CL;
        end

        printRow(name, seg, h1, h2, speedStr, Thrust_seg, CL_seg);
    end

    function printCruiseSegmentRow(name, seg)
        h1 = getFieldOr(seg,'Altitude',NaN);
        h2 = NaN;

        if isfield(seg,'StepAlt') && ~isempty(seg.StepAlt)
            h1 = seg.StepAlt(1);
            h2 = seg.StepAlt(end);
        end

        if isfield(seg,'StepMach') && ~isempty(seg.StepMach)
            speedStr = sprintf('M %.3f', mean(seg.StepMach,'omitnan'));
        else
            speedStr = '-';
        end

        Thrust_seg = NaN;
        CL_seg = NaN;

        if isfield(seg,'StepTreq') && ~isempty(seg.StepTreq)
            Thrust_seg = mean(seg.StepTreq,'omitnan') / 1000;
        elseif isfield(seg,'Treq')
            Thrust_seg = seg.Treq / 1000;
        end

        if isfield(seg,'StepCL') && ~isempty(seg.StepCL)
            CL_seg = mean(seg.StepCL,'omitnan');
        elseif isfield(seg,'CL')
            CL_seg = seg.CL;
        end

        printRow(name, seg, h1, h2, speedStr, Thrust_seg, CL_seg);
    end

end

function plotMissionHistory(Mission, M_TO)

[t_hist, h_hist, m_hist, labels] = buildMissionHistory(Mission, M_TO);

figure(12); clf;
tiledlayout(2,1)

nexttile
plot(t_hist/60, h_hist, 'LineWidth', 1.5)
xlabel('Cumulative time (min)')
ylabel('Altitude (m)')
title('Mission altitude history')
grid on
hold on
for i = 1:numel(labels)
    xline(labels(i).time_s/60, '--');
end

nexttile
plot(t_hist/60, m_hist, 'LineWidth', 1.5)
xlabel('Cumulative time (min)')
ylabel('Aircraft mass (kg)')
title('Mission mass history')
grid on

end

function [t_hist, h_hist, m_hist, labels] = buildMissionHistory(Mission, M_TO)

t_hist = 0;
h_hist = 0;
m_hist = M_TO;

t_now = 0;
m_now = M_TO;

labels = struct('name',{},'time_s',{});

    function addSimpleSegment(name, seg, h1, h2)
        time = getFieldOr(seg,'Time',0);
        fuel = getFieldOr(seg,'Fuel',0);

        t_seg = [t_now; t_now + time];
        h_seg = [h1; h2];
        m_seg = [m_now; m_now - fuel];

        t_hist = [t_hist; t_seg(2:end)]; %#ok<AGROW>
        h_hist = [h_hist; h_seg(2:end)]; %#ok<AGROW>
        m_hist = [m_hist; m_seg(2:end)]; %#ok<AGROW>

        t_now = t_now + time;
        m_now = m_now - fuel;

        labels(end+1).name = name; %#ok<AGROW>
        labels(end).time_s = t_now;
    end

    function addStepSegment(name, seg)
        if ~isfield(seg,'StepTime') || isempty(seg.StepTime)
            addSimpleSegment(name, seg, NaN, NaN);
            return
        end

        t_local = t_now + cumsum(seg.StepTime(:));

        if isfield(seg,'StepAlt') && ~isempty(seg.StepAlt)
            h_local = seg.StepAlt(:);
        else
            h_local = NaN(size(t_local));
        end

        if isfield(seg,'StepFuel') && ~isempty(seg.StepFuel)
            m_local = m_now - cumsum(seg.StepFuel(:));
        else
            m_local = m_now * ones(size(t_local));
        end

        t_hist = [t_hist; t_local]; %#ok<AGROW>
        h_hist = [h_hist; h_local]; %#ok<AGROW>
        m_hist = [m_hist; m_local]; %#ok<AGROW>

        t_now = t_local(end);
        m_now = m_local(end);

        labels(end+1).name = name; %#ok<AGROW>
        labels(end).time_s = t_now;
    end

addSimpleSegment('TaxiOut', Mission.TaxiOut, 0, 0);
addSimpleSegment('TakeoffGround', Mission.TakeoffGround, 0, 0);
addSimpleSegment('InitialClimb', Mission.InitialClimb, 0, getFieldOr(Mission.InitialClimb,'h_end',NaN));

addStepSegment('Climb1', Mission.Climb1);
addStepSegment('Climb2', Mission.Climb2);
addStepSegment('Climb3', Mission.Climb3);
addStepSegment('Cruise', Mission.Cruise);
addStepSegment('Descent1', Mission.Descent1);
addStepSegment('Descent2', Mission.Descent2);
addStepSegment('Descent3', Mission.Descent3);

addSimpleSegment('ApproachLanding', Mission.ApproachLanding, 1500/SI.ft, 0);
addSimpleSegment('TaxiIn', struct('Time',20*60,'Fuel',0), 0, 0);

addStepSegment('AltClimb', Mission.Alternate.Climb);
addStepSegment('AltCruise', Mission.Alternate.Cruise);
addStepSegment('AltDescent', Mission.Alternate.Descent);

addSimpleSegment('AltApproach', Mission.Alternate.Approach, 0, 0);
addSimpleSegment('Loiter', Mission.Loiter, 1500/SI.ft, 1500/SI.ft);
addSimpleSegment('Contingency', Mission.Contingency, 1500/SI.ft, 1500/SI.ft);

end

function val = getFieldOr(S, fieldName, fallback)
if isstruct(S) && isfield(S, fieldName) && ~isempty(S.(fieldName))
    val = S.(fieldName);
else
    val = fallback;
end
end

function M = estimateMachFromLoiter(~)
M = NaN;
end

