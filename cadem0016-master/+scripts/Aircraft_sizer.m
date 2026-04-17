%% ExampleSizing_Baseline
% Sizes a single aircraft configuration, plots it, and prints a clean
% summary of geometry, mass, aero and economics for the selected fleet case.

clear; clc; close all;

%% ------------------------- Instantiate aircraft -------------------------
ADP = B777.ADP();
ADP.TLAR = cast.TLAR.B777F();   % top level aircraft requirements

%% ------------------------- Hyper-parameters ----------------------------
ADP.TLAR.M_c = 0.845;
ADP.Fleet_size = 8;
ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size; % NOT A HYPERPARAMETER

ADP.AR = 8;
ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
ADP.WingLoading = 7000;
ADP.cruise_altitude = 11626;

%% ---------------------- Geometric parameters ---------------------------
ADP.KinkPos = 10;   % spanwise position of TE kink in wing planform

[ADP.CabinLength, ADP.CabinRadius, ADP.L_total] = ...
    B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ...
                             ADP.CockpitLength, ADP.D_max);

ADP.WingPos = 0.44 * ADP.L_total;   % wing position [m] from nose
ADP.V_HT    = 0.75;                 % horizontal tail volume coefficient
ADP.V_VT    = 0.07;                 % vertical tail volume coefficient
ADP.HtpPos  = 0.85 * ADP.L_total;   % HTP position [m] from nose
ADP.VtpPos  = 0.82 * ADP.L_total;   % VTP position [m] from nose

%% ------------------------ Class-I estimates ----------------------------
ADP.MTOM    = 3.35 * ADP.TLAR.Payload;  % initial guess
ADP.Mf_Fuel = 0.19;                     % max fuel mass fraction
ADP.Mf_res  = 0.03;                     % reserve fuel mass fraction
ADP.Mf_Ldg  = 0.68;                     % landing mass fraction
ADP.Mf_TOC  = 0.97;                     % top-of-climb mass fraction

%% ------------------------------ Sizing ---------------------------------
ADP = B777.Size(ADP);

%% ------------------- Build geometry and masses -------------------------
[B7Geom, B7Mass] = B777.BuildGeometry(ADP);
d = B7Mass.GetData;

%% ------------------------- Mission analysis ----------------------------
[BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, Mission, CriticalTW, CriticalWS] = ...
    B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

BlockTime_hr = MissionTime / 3600;

%% -------------------------- Cost analysis ------------------------------
FuelType = 'JetA1';

FlightsPerYear      = ADP.TLAR.FlightsPerYear;
AnnualUtilisationHr = ADP.TLAR.FlightHours;
NumLandings         = 1;
NumCycles           = 1;

FleetSize      = ADP.Fleet_size;
LifecycleYears = 20;

% Placeholder acquisition cost model:
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
    'FleetSize', FleetSize, ...
    'LifecycleYears', LifecycleYears, ...
    'ParkingDaysPerYear', ADP.TLAR.ParkingDays, ...
    'NavChargePerFlight', 0, ...
    'UseImprovedMaint', false, ...
    'NumEngines', 2, ...
    'EngineCost', 0);

%% ---------------------- Per-aircraft outputs ---------------------------
HullValue       = Econ.breakdown.HullValue;
CrewCost        = Econ.breakdown.CrewAnnual;
LandingFee      = Econ.breakdown.LandingFeeTrip;
LandingAnnual   = Econ.breakdown.LandingAnnual;
ParkingFee      = Econ.breakdown.ParkingAnnual;
FuelCost        = Econ.breakdown.FuelTrip;
FuelAnnual      = Econ.breakdown.FuelAnnual;
MaintFixed      = Econ.breakdown.MaintFixedAnnual;
MaintVar        = Econ.breakdown.MaintVarAnnual;
MaintAnnual     = Econ.breakdown.MaintAnnual;
InsuranceCost   = Econ.breakdown.InsuranceAnnual;
NavigationCost  = Econ.breakdown.NavigationAnnual;

COC_Annual      = Econ.COC_annual;
DOC_Annual      = Econ.DOC_annual;

%% ---------------------- Fleet annual outputs ---------------------------
FleetCrewCost      = Econ.breakdown.FleetCrewAnnual;
FleetLandingCost   = Econ.breakdown.FleetLandingAnnual;
FleetParkingCost   = Econ.breakdown.FleetParkingAnnual;
FleetFuelCost      = Econ.breakdown.FleetFuelAnnual;
FleetHullValue     = FleetSize * HullValue;
FleetMaintCost     = Econ.breakdown.FleetMaintAnnual;
FleetInsuranceCost = Econ.breakdown.FleetInsuranceAnnual;
FleetNavCost       = Econ.breakdown.FleetNavAnnual;

FleetCapitalCost   = Econ.FleetCapitalCost;
FleetTotalCOC      = Econ.FleetCOC_annual;
FleetTotalDOC      = Econ.FleetDOC_annual;

%% ---------------------- Fleet lifecycle outputs ------------------------
FleetOperatingCost20yr_COC = Econ.FleetCOC_20yr;
FleetOperatingCost20yr_DOC = Econ.FleetDOC_20yr;
FleetLifecycleCost20yr_COC = Econ.FleetLifecycleCostCOC;
FleetLifecycleCost20yr_DOC = Econ.FleetLifecycleCostDOC;

% Main optimization objective
TotalCost = Econ.Objective;

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
fprintf('Cruise altitude                    : %8.1f m\n', ADP.cruise_altitude);
fprintf('Flights per year                   : %8.1f\n', FlightsPerYear);
fprintf('Annual utilisation                 : %8.1f hr/year\n', AnnualUtilisationHr);
fprintf('Lifecycle                          : %8d years\n', LifecycleYears);

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
fprintf('Trip fuel                          : %8.1f t\n', TripFuel/1e3);
fprintf('Reserve fuel                       : %8.1f t\n', ResFuel/1e3);
fprintf('Block fuel per mission             : %8.1f t\n', BlockFuel/1e3);
fprintf('Mission time                       : %8.2f hr\n', MissionTime/3600);

% --- see all wing entries (please dont remove my code keeps breaking) ---
wingIdxAll = contains(d(:,1), "Wing");

disp("---- ALL WING ENTRIES ----")
disp(d(wingIdxAll,:))

% --- pick correct one ---
wingIdx = strcmp(d(:,1), "Wing_Structural");

if any(wingIdx)
    wing_mass = str2double(d{wingIdx,2});
    fprintf('Wing mass                          : %8.1f t\n', wing_mass/1e3);
else
    warning("Wing_Structural not found!")
end

fprintf('\n--- Aerodynamics -------------------------------------------\n');
fprintf('CD0                                : %8.4f\n', ADP.AeroPolar.CD(0));
fprintf('CD at CL = 0.5                     : %8.4f\n', ADP.AeroPolar.CD(0.5));
fprintf('Mf_TOC (mission analysis)          : %8.4f\n', Mf_TOC);

fprintf('\n--- Economics: Per Aircraft --------------------------------\n');
fprintf('Investment / aircraft              : %8.3f M$\n', TotalInvestment/1e6);
fprintf('Hull value                         : %8.3f M$\n', HullValue/1e6);
fprintf('COC                                : %8.3f M$/yr\n', COC_Annual/1e6);
fprintf('DOC                                : %8.3f M$/yr\n', DOC_Annual/1e6);
fprintf('COC                                : %8.3f k$/flight\n', Econ.COC_trip/1e3);
fprintf('DOC                                : %8.3f k$/flight\n', Econ.DOC_trip/1e3);
fprintf('Crew cost                          : %8.3f M$/yr\n', CrewCost/1e6);
fprintf('Landing fee                        : %8.3f k$/flight\n', LandingFee/1e3);
fprintf('Landing fees                       : %8.3f M$/yr\n', LandingAnnual/1e6);
fprintf('Parking fee                        : %8.3f M$/yr\n', ParkingFee/1e6);
fprintf('Fuel cost                          : %8.3f k$/flight\n', FuelCost/1e3);
fprintf('Fuel cost                          : %8.3f M$/yr\n', FuelAnnual/1e6);
fprintf('Fixed maintenance                  : %8.3f M$/yr\n', MaintFixed/1e6);
fprintf('Variable maintenance               : %8.3f M$/yr\n', MaintVar/1e6);
fprintf('Total maintenance                  : %8.3f M$/yr\n', MaintAnnual/1e6);
fprintf('Insurance cost                     : %8.3f M$/yr\n', InsuranceCost/1e6);
fprintf('Navigation cost                    : %8.3f M$/yr\n', NavigationCost/1e6);

fprintf('\n--- Economics: Entire Fleet --------------------------------\n');
fprintf('Fleet capital cost                 : %8.3f M$\n', FleetCapitalCost/1e6);
fprintf('Fleet COC                          : %8.3f M$/yr\n', FleetTotalCOC/1e6);
fprintf('Fleet DOC                          : %8.3f M$/yr\n', FleetTotalDOC/1e6);
fprintf('Fleet crew cost                    : %8.3f M$/yr\n', FleetCrewCost/1e6);
fprintf('Fleet landing fees                 : %8.3f M$/yr\n', FleetLandingCost/1e6);
fprintf('Fleet parking fees                 : %8.3f M$/yr\n', FleetParkingCost/1e6);
fprintf('Fleet fuel cost                    : %8.3f M$/yr\n', FleetFuelCost/1e6);
fprintf('Fleet hull value                   : %8.3f M$\n', FleetHullValue/1e6);
fprintf('Fleet maintenance                  : %8.3f M$/yr\n', FleetMaintCost/1e6);
fprintf('Fleet insurance cost               : %8.3f M$/yr\n', FleetInsuranceCost/1e6);
fprintf('Fleet navigation cost              : %8.3f M$/yr\n', FleetNavCost/1e6);

fprintf('\n--- Economics: 20-Year Lifecycle ---------------------------\n');
fprintf('Fleet operating cost (COC, 20 yr)  : %8.3f M$\n', FleetOperatingCost20yr_COC/1e6);
fprintf('Fleet operating cost (DOC, 20 yr)  : %8.3f M$\n', FleetOperatingCost20yr_DOC/1e6);
fprintf('Fleet lifecycle cost (COC, 20 yr)  : %8.3f M$\n', FleetLifecycleCost20yr_COC/1e6);
fprintf('Fleet lifecycle cost (DOC, 20 yr)  : %8.3f M$\n', FleetLifecycleCost20yr_DOC/1e6);
fprintf('Optimization objective             : %8.3f M$\n', TotalCost/1e6);

%% ---------------------- Cost summary table -----------------------------
CostNames = { ...
    'Crew Cost / aircraft / year'
    'Landing Fee / aircraft / flight'
    'Landing Fees / aircraft / year'
    'Parking Fee / aircraft / year'
    'Fuel Cost / aircraft / flight'
    'Fuel Cost / aircraft / year'
    'Hull Value / aircraft'
    'Investment / aircraft'
    'Maint. Fixed / aircraft / year'
    'Maint. Variable / aircraft / year'
    'Maint. Total / aircraft / year'
    'Insurance / aircraft / year'
    'Navigation / aircraft / year'
    'COC / aircraft / year'
    'DOC / aircraft / year'
    'Fleet Crew Cost / year'
    'Fleet Landing Fees / year'
    'Fleet Parking Fees / year'
    'Fleet Fuel Cost / year'
    'Fleet Hull Value'
    'Fleet Maintenance / year'
    'Fleet Insurance / year'
    'Fleet Navigation / year'
    'Fleet Total COC / year'
    'Fleet Total DOC / year'
    'Fleet Capital Cost'
    'Fleet Operating Cost / 20 yr (COC)'
    'Fleet Operating Cost / 20 yr (DOC)'
    'Fleet Lifecycle Cost / 20 yr (COC)'
    'Fleet Lifecycle Cost / 20 yr (DOC)'
    'Optimization Objective'};

CostValues_MUSD = [ ...
    CrewCost
    LandingFee
    LandingAnnual
    ParkingFee
    FuelCost
    FuelAnnual
    HullValue
    TotalInvestment
    MaintFixed
    MaintVar
    MaintAnnual
    InsuranceCost
    NavigationCost
    COC_Annual
    DOC_Annual
    FleetCrewCost
    FleetLandingCost
    FleetParkingCost
    FleetFuelCost
    FleetHullValue
    FleetMaintCost
    FleetInsuranceCost
    FleetNavCost
    FleetTotalCOC
    FleetTotalDOC
    FleetCapitalCost
    FleetOperatingCost20yr_COC
    FleetOperatingCost20yr_DOC
    FleetLifecycleCost20yr_COC
    FleetLifecycleCost20yr_DOC
    TotalCost] / 1e6;

CostSummary = table(CostNames, CostValues_MUSD, ...
    'VariableNames', {'Cost_Item', 'Value_MUSD'});

disp(' ');
disp('Cost Summary Table:');
disp(CostSummary);

%% ---------------------- Engine properties ------------------------------
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
        fprintf('%-30s : %s\n', props{i}, string(val));
    else
        fprintf('%-30s : [non-displayable type]\n', props{i});
    end
end

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

printRow('ApproachLanding', Mission.ApproachLanding, 1500 / SI.ft, 0, ...
    sprintf('%.1f m/s', getFieldOr(Mission.ApproachLanding,'V',NaN)), ...
    getFieldOr(Mission.ApproachLanding,'Treq',NaN)/1000, ...
    getFieldOr(Mission.ApproachLanding,'CL',NaN));

% ---------------- Reserve mission ----------------
printVerticalSegmentRow('AltClimb', Mission.Alternate.Climb);
printCruiseSegmentRow('AltCruise', Mission.Alternate.Cruise);
printVerticalSegmentRow('AltDescent', Mission.Alternate.Descent);

printRow('Loiter', Mission.Loiter, 1500 / SI.ft, 1500 / SI.ft, ...
    loiterSpeedString(Mission.Loiter), ...
    getFieldOr(Mission.Loiter,'Treq',NaN)/1000, ...
    getFieldOr(Mission.Loiter,'CL',NaN));

printVerticalSegmentRow('AltApprDescent', Mission.Alternate.ApproachDescent);

printRow('AltApproach', Mission.Alternate.Approach, 0, 0, ...
    sprintf('%.1f m/s', getFieldOr(Mission.Alternate.Approach,'V',NaN)), ...
    getFieldOr(Mission.Alternate.Approach,'Treq',NaN)/1000, ...
    getFieldOr(Mission.Alternate.Approach,'CL',NaN));

fprintf('%s\n', repmat('-',1,135));

TripFuel_rep  = getNestedFieldOr(Mission, {'Total','TripFuel'}, NaN);
ResFuel_rep   = getNestedFieldOr(Mission, {'Total','ResFuel'}, NaN);
BlockFuel_rep = getNestedFieldOr(Mission, {'Total','BlockFuel'}, NaN);

if isnan(TripFuel_rep)
    TripFuel_rep = getFieldOr(Mission.TaxiOut,'Fuel',0) + ...
                   getFieldOr(Mission.TakeoffGround,'Fuel',0) + ...
                   getFieldOr(Mission.InitialClimb,'Fuel',0) + ...
                   getFieldOr(Mission.Climb1,'Fuel',0) + ...
                   getFieldOr(Mission.Climb2,'Fuel',0) + ...
                   getFieldOr(Mission.Climb3,'Fuel',0) + ...
                   getFieldOr(Mission.Cruise,'Fuel',0) + ...
                   getFieldOr(Mission.Descent1,'Fuel',0) + ...
                   getFieldOr(Mission.Descent2,'Fuel',0) + ...
                   getFieldOr(Mission.Descent3,'Fuel',0) + ...
                   getFieldOr(Mission.ApproachLanding,'Fuel',0);
end

if isnan(ResFuel_rep)
    ResFuel_rep = getFieldOr(Mission.Alternate.Climb,'Fuel',0) + ...
                  getFieldOr(Mission.Alternate.Cruise,'Fuel',0) + ...
                  getFieldOr(Mission.Alternate.Descent,'Fuel',0) + ...
                  getFieldOr(Mission.Loiter,'Fuel',0) + ...
                  getFieldOr(Mission.Alternate.ApproachDescent,'Fuel',0) + ...
                  getFieldOr(Mission.Alternate.Approach,'Fuel',0) + ...
                  getNestedFieldOr(Mission, {'Total','ContingencyFuel'}, 0);
end

if isnan(BlockFuel_rep)
    BlockFuel_rep = TripFuel_rep + ResFuel_rep;
end

fprintf('Trip fuel                 = %.1f kg\n', TripFuel_rep);
fprintf('Reserve fuel              = %.1f kg\n', ResFuel_rep);
fprintf('Block fuel                = %.1f kg\n', BlockFuel_rep);

AirborneTime_rep = getNestedFieldOr(Mission, {'Total','AirborneDesignMissionTime'}, NaN);
DesignTime_rep   = getNestedFieldOr(Mission, {'Total','DesignMissionTime'}, NaN);
TotalTime_rep    = getNestedFieldOr(Mission, {'Total','TotalMissionTime'}, NaN);

if ~isnan(AirborneTime_rep)
    fprintf('Airborne design time      = %.2f hr\n', AirborneTime_rep/3600);
end
if ~isnan(DesignTime_rep)
    fprintf('Design mission time       = %.2f hr\n', DesignTime_rep/3600);
end
if ~isnan(TotalTime_rep)
    fprintf('Total mission time        = %.2f hr\n', TotalTime_rep/3600);
end

if isfield(Mission,'Total') && isfield(Mission.Total,'ContingencyFuel')
    fprintf('Contingency fuel carried  = %.1f kg\n', Mission.Total.ContingencyFuel);
end

fprintf('========================================================================================================\n');

    function printVerticalSegmentRow(name, seg)
        h1 = NaN;
        h2 = NaN;
        speedStr = '-';

        if isfield(seg,'StartAltitude') && isfield(seg,'EndAltitude')
            h1 = seg.StartAltitude;
            h2 = seg.EndAltitude;
        elseif isfield(seg,'StepAlt') && ~isempty(seg.StepAlt)
            if numel(seg.StepAlt) == 1
                h1 = seg.StepAlt(1);
                h2 = seg.StepAlt(1);
            else
                dh = mean(abs(diff(seg.StepAlt)), 'omitnan');
                if isempty(dh) || isnan(dh), dh = 0; end

                if seg.StepAlt(end) >= seg.StepAlt(1)
                    h1 = seg.StepAlt(1) - dh/2;
                    h2 = seg.StepAlt(end) + dh/2;
                else
                    h1 = seg.StepAlt(1) + dh/2;
                    h2 = seg.StepAlt(end) - dh/2;
                end
            end
        end

        if isfield(seg,'StepMach') && ~isempty(seg.StepMach)
            speedStr = sprintf('M %.2f', mean(seg.StepMach,'omitnan'));
        elseif isfield(seg,'StepV') && ~isempty(seg.StepV)
            speedStr = sprintf('%.1f m/s', mean(seg.StepV,'omitnan'));
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
        h1 = getFieldOr(seg,'StartAltitude', getFieldOr(seg,'Altitude',NaN));
        h2 = getFieldOr(seg,'EndAltitude', NaN);

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

[t_hist, h_hist, m_hist, ~] = buildMissionHistory(Mission, M_TO);

figure(12); clf;
tiledlayout(2,1)

nexttile
plot(t_hist/60, h_hist, 'LineWidth', 1.8)
xlabel('Cumulative time (min)')
ylabel('Altitude (m)')
title('Mission Altitude History')
grid off

nexttile
plot(t_hist/60, m_hist, 'LineWidth', 1.8)
xlabel('Cumulative time (min)')
ylabel('Aircraft mass (kg)')
title('Mission Mass History')
grid off

end

%% ---------------- Mass history ----------------
nexttile
plot(t_hist/60, m_hist, 'LineWidth', 1.8)
xlabel('Cumulative time (min)')
ylabel('Aircraft mass (kg)')
title('Mission Mass History')
grid on
hold on

for i = 1:numel(labels)
    x_lab = labels(i).time_s / 60;

    [~, idx] = min(abs(t_hist/60 - x_lab));
    y_lab = m_hist(idx);

    text(x_lab, y_lab, ['  ' labels(i).name], ...
        'FontSize', 8, ...
        'Rotation', 0, ...
        'VerticalAlignment', 'bottom', ...
        'Interpreter', 'none');
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

addSimpleSegment('ApproachLanding', Mission.ApproachLanding, 1500 / SI.ft, 0);
addSimpleSegment('TaxiIn', struct('Time',20*60,'Fuel',0), 0, 0);

addStepSegment('AltClimb', Mission.Alternate.Climb);
addStepSegment('AltCruise', Mission.Alternate.Cruise);
addStepSegment('AltDescent', Mission.Alternate.Descent);
addSimpleSegment('Loiter', Mission.Loiter, 1500 / SI.ft, 1500 / SI.ft);
addStepSegment('AltApprDescent', Mission.Alternate.ApproachDescent);
addSimpleSegment('AltApproach', Mission.Alternate.Approach, 0, 0);
addSimpleSegment('TaxiInFinal', struct('Time',20*60,'Fuel',0), 0, 0);

end

function val = getFieldOr(S, fieldName, fallback)
if isstruct(S) && isfield(S, fieldName) && ~isempty(S.(fieldName))
    val = S.(fieldName);
else
    val = fallback;
end
end

function val = getNestedFieldOr(S, fieldPath, fallback)
val = fallback;

try
    current = S;
    for k = 1:numel(fieldPath)
        if isstruct(current) && isfield(current, fieldPath{k})
            current = current.(fieldPath{k});
        else
            return
        end
    end

    if ~isempty(current)
        val = current;
    end
catch
    val = fallback;
end
end

function M = estimateMachFromLoiter(seg)
M = getFieldOr(seg, 'Mach', NaN);
end

function s = loiterSpeedString(seg)
if isfield(seg,'V') && ~isempty(seg.V) && ~isnan(seg.V)
    s = sprintf('%.1f m/s', seg.V);
elseif isfield(seg,'Mach') && ~isempty(seg.Mach) && ~isnan(seg.Mach)
    s = sprintf('M %.2f', seg.Mach);
else
    s = '-';
end
end