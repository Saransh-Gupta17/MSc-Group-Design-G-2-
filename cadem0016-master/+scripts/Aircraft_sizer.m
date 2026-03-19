%% ExampleSizing_Baseline
% Sizes a single aircraft configuration, plots it, and prints a clean
% summary of geometry, mass, aero and economics for the selected fleet case.

clear; clc; close all;

%% ------------------------- Instantiate aircraft -------------------------
ADP = B777.ADP();
ADP.TLAR = cast.TLAR.B777F();   % top level aircraft requirements

%% ------------------------- Hyper-parameters ----------------------------
ADP.TLAR.M_c = 0.84;
ADP.Fleet_size = 6;
ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;

ADP.AR = 9.5;
ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
ADP.WingLoading = (347815*9.81)/(473.3*cosd(31.6));
ADP.cruise_altitude = 12000;

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
[BlockFuel, ~, ~, ~, ~] = B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

%% -------------------------- Cost analysis ------------------------------
FuelType = 'JetA1';

[CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, ...
    MaintFixed, MaintVar, InsuranceCost, TotalCost] = ...
    B777.Economics(ADP.MTOM, ADP.Span, BlockFuel, ...
                   ADP.TLAR.FlightHours, ADP.TLAR.ParkingDays, FuelType);

FlightsPerYear = ADP.TLAR.FlightsPerYear;

% Convert to annual whole-fleet values
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

% --- DEBUG: see all wing entries ---
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
fprintf('Crew cost                          : %8.3f M$/yr\n', CrewCost/1e6);
fprintf('Landing fee                        : %8.3f M$/flight\n', LandingFee/1e6);
fprintf('Parking fee                        : %8.3f M$/yr\n', ParkingFee/1e6);
fprintf('Fuel cost                          : %8.3f M$/flight\n', FuelCost/1e6);
fprintf('Hull value                         : %8.3f M$\n', HullValue/1e6);
fprintf('Fixed maintenance                  : %8.3f M$/yr\n', MaintFixed/1e6);
fprintf('Variable maintenance               : %8.3f M$/yr\n', MaintVar/1e6);
fprintf('Insurance cost                     : %8.3f M$/yr\n', InsuranceCost/1e6);
fprintf('Total DOC (raw function output)    : %8.3f M$\n', TotalCost/1e6);

fprintf('\n--- Economics: Entire Fleet --------------------------------\n');
fprintf('Fleet crew cost                    : %8.3f M$/yr\n', FleetCrewCost/1e6);
fprintf('Fleet landing fees                 : %8.3f M$/yr\n', FleetLandingCost/1e6);
fprintf('Fleet parking fees                 : %8.3f M$/yr\n', FleetParkingCost/1e6);
fprintf('Fleet fuel cost                    : %8.3f M$/yr\n', FleetFuelCost/1e6);
fprintf('Fleet hull value                   : %8.3f M$\n', FleetHullValue/1e6);
fprintf('Fleet fixed maintenance            : %8.3f M$/yr\n', FleetMaintFixed/1e6);
fprintf('Fleet variable maintenance         : %8.3f M$/yr\n', FleetMaintVar/1e6);
fprintf('Fleet insurance cost               : %8.3f M$/yr\n', FleetInsuranceCost/1e6);
fprintf('Fleet total direct operating cost  : %8.3f M$/yr\n', FleetTotalDOC/1e6);

fprintf('============================================================\n');

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