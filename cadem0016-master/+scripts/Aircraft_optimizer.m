%% ExampleSizing_TradeStudies
% Runs trade studies using a baseline aircraft setup.

clear; clc; close all;

%% ------------------------- Instantiate aircraft -------------------------
ADP = B777.ADP();
ADP.TLAR = cast.TLAR.B777F();

%% ------------------------- Hyper-parameters ----------------------------
ADP.TLAR.M_c = 0.84;
ADP.Fleet_size = 6;
ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;

ADP.AR = 9.5;
ADP.ThrustToWeightRatio = (513e3*2)/(347815*9.81);
ADP.WingLoading = (347815*9.81)/(473.3*cosd(31.6));
ADP.cruise_altitude = 12000;

%% ---------------------- Geometric parameters ---------------------------
ADP.KinkPos = 10;

[ADP.CabinLength, ADP.CabinRadius, ADP.L_total] = ...
    B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ...
                             ADP.CockpitLength, ADP.D_max);

ADP.WingPos = 0.44 * ADP.L_total;
ADP.V_HT    = 0.75;
ADP.V_VT    = 0.07;
ADP.HtpPos  = 0.85 * ADP.L_total;
ADP.VtpPos  = 0.82 * ADP.L_total;

%% ------------------------ Class-I estimates ----------------------------
ADP.MTOM    = 3.35 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.19;
ADP.Mf_res  = 0.03;
ADP.Mf_Ldg  = 0.68;
ADP.Mf_TOC  = 0.97;

%% ------------------------- Baseline sizing -----------------------------
ADP = B777.Size(ADP);

% Save a clean baseline
ADP0 = ADP;

%% ==============================================================
%% Trade Study 1: MTOM and Block Fuel vs Wing Span
%% ==============================================================
Spans = 50:5:100;

mtoms = zeros(size(Spans));
fuels = zeros(size(Spans));

for i = 1:length(Spans)
    ADPi = ADP0;
    ADPi.Span = Spans(i);
    ADPi = B777.Size(ADPi);

    mtoms(i) = ADPi.MTOM;
    fuels(i) = ADPi.Mf_Fuel * ADPi.MTOM;
end

figure(1); clf;
tt = tiledlayout(2,1);

nexttile;
plot(Spans, mtoms/1e3, '-s', 'LineWidth', 1.5)
xlabel('Span [m]')
ylabel('MTOM [t]')
title('MTOM vs Wing Span')
grid on

nexttile;
plot(Spans, fuels/1e3, '-o', 'LineWidth', 1.5)
xlabel('Span [m]')
ylabel('Fuel Mass [t]')
title('Fuel Mass vs Wing Span')
grid on

%% ==============================================================
%% Trade Study 2: MTOM and Block Fuel vs Fleet Size
%% ==============================================================
FleetSizes = 6:1:14;

mtoms_fleet = zeros(size(FleetSizes));
blockfuel_per_aircraft = zeros(size(FleetSizes));
total_blockfuel_fleet = zeros(size(FleetSizes));

for i = 1:length(FleetSizes)
    ADPi = ADP0;

    ADPi.Fleet_size = FleetSizes(i);
    ADPi.TLAR.Payload = ADPi.Total_Payload / ADPi.Fleet_size;

    [ADPi.CabinLength, ADPi.CabinRadius, ADPi.L_total] = ...
        B777.geom.fuselage_sizer(ADPi.Fleet_size, ADPi.Pallet_size, ...
                                 ADPi.CockpitLength, ADPi.D_max);

    ADPi.WingPos = 0.44 * ADPi.L_total;
    ADPi.HtpPos  = 0.85 * ADPi.L_total;
    ADPi.VtpPos  = 0.82 * ADPi.L_total;

    ADPi = B777.Size(ADPi);

    [BlockFuel, ~, ~, ~, ~] = B777.MissionAnalysis(ADPi, ADPi.TLAR.Range, ADPi.MTOM);

    mtoms_fleet(i) = ADPi.MTOM;
    blockfuel_per_aircraft(i) = BlockFuel;
    total_blockfuel_fleet(i) = ADPi.Fleet_size * BlockFuel;
end

figure(2); clf;
tt = tiledlayout(3,1);

nexttile;
plot(FleetSizes, mtoms_fleet/1e3, '-s', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('MTOM per Aircraft [t]')
title('MTOM per Aircraft vs Fleet Size')
grid on

nexttile;
plot(FleetSizes, blockfuel_per_aircraft/1e3, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Block Fuel per Aircraft [t]')
title('Block Fuel per Aircraft vs Fleet Size')
grid on

nexttile;
plot(FleetSizes, total_blockfuel_fleet/1e3, '-d', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Total Fleet Block Fuel [t]')
title('Total Fleet Block Fuel vs Fleet Size')
grid on

%% ==============================================================
%% Trade Study 3: Entire Fleet Annual DOC vs Fleet Size
%% ==============================================================
FleetSizes = 5:1:12;

FlightHours    = ADP0.TLAR.FlightHours;
ParkingDays    = ADP0.TLAR.ParkingDays;
FlightsPerYear = ADP0.TLAR.FlightsPerYear;
FuelType       = 'JetA1';

fleet_crew_cost      = zeros(size(FleetSizes));
fleet_landing_cost   = zeros(size(FleetSizes));
fleet_parking_cost   = zeros(size(FleetSizes));
fleet_fuel_cost      = zeros(size(FleetSizes));
fleet_hull_value     = zeros(size(FleetSizes));
fleet_maint_fixed    = zeros(size(FleetSizes));
fleet_maint_var      = zeros(size(FleetSizes));
fleet_insurance_cost = zeros(size(FleetSizes));
fleet_total_cost     = zeros(size(FleetSizes));

for i = 1:length(FleetSizes)
    ADPi = ADP0;

    ADPi.Fleet_size = FleetSizes(i);
    ADPi.TLAR.Payload = ADPi.Total_Payload / ADPi.Fleet_size;

    [ADPi.CabinLength, ADPi.CabinRadius, ADPi.L_total] = ...
        B777.geom.fuselage_sizer(ADPi.Fleet_size, ADPi.Pallet_size, ...
                                 ADPi.CockpitLength, ADPi.D_max);

    ADPi.WingPos = 0.44 * ADPi.L_total;
    ADPi.HtpPos  = 0.85 * ADPi.L_total;
    ADPi.VtpPos  = 0.82 * ADPi.L_total;

    ADPi = B777.Size(ADPi);

    [BlockFuel, ~, ~, ~, ~] = B777.MissionAnalysis(ADPi, ADPi.TLAR.Range, ADPi.MTOM);

    [CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, ...
        MaintFixed, MaintVar, InsuranceCost, ~] = ...
        B777.Economics(ADPi.MTOM, ADPi.Span, BlockFuel, ...
                       FlightHours, ParkingDays, FuelType);

    fleet_crew_cost(i)      = ADPi.Fleet_size * CrewCost;
    fleet_landing_cost(i)   = ADPi.Fleet_size * LandingFee * FlightsPerYear;
    fleet_parking_cost(i)   = ADPi.Fleet_size * ParkingFee;
    fleet_fuel_cost(i)      = ADPi.Fleet_size * FuelCost * FlightsPerYear;
    fleet_hull_value(i)     = ADPi.Fleet_size * HullValue;
    fleet_maint_fixed(i)    = ADPi.Fleet_size * MaintFixed;
    fleet_maint_var(i)      = ADPi.Fleet_size * MaintVar;
    fleet_insurance_cost(i) = ADPi.Fleet_size * InsuranceCost;

    fleet_total_cost(i) = fleet_crew_cost(i) + fleet_landing_cost(i) + ...
                          fleet_parking_cost(i) + fleet_fuel_cost(i) + ...
                          fleet_maint_fixed(i) + fleet_maint_var(i) + ...
                          fleet_insurance_cost(i);
end

figure(3); clf;
tt = tiledlayout(4,2);

nexttile;
plot(FleetSizes, fleet_crew_cost/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Crew Cost [$M/yr]')
title('Entire Fleet Crew Cost')
grid on

nexttile;
plot(FleetSizes, fleet_landing_cost/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Landing Fees [$M/yr]')
title('Entire Fleet Landing Fees')
grid on

nexttile;
plot(FleetSizes, fleet_parking_cost/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Parking Fees [$M/yr]')
title('Entire Fleet Parking Fees')
grid on

nexttile;
plot(FleetSizes, fleet_fuel_cost/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Fuel Cost [$M/yr]')
title('Entire Fleet Fuel Cost')
grid on

nexttile;
plot(FleetSizes, fleet_hull_value/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Hull Value [$M]')
title('Entire Fleet Hull Value')
grid on

nexttile;
plot(FleetSizes, fleet_maint_fixed/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Fixed Maintenance [$M/yr]')
title('Entire Fleet Fixed Maintenance')
grid on

nexttile;
plot(FleetSizes, fleet_maint_var/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Variable Maintenance [$M/yr]')
title('Entire Fleet Variable Maintenance')
grid on

nexttile;
plot(FleetSizes, fleet_insurance_cost/1e6, '-o', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Insurance Cost [$M/yr]')
title('Entire Fleet Insurance')
grid on

figure(4); clf;
plot(FleetSizes, fleet_total_cost/1e6, '-s', 'LineWidth', 1.5)
xlabel('Fleet Size')
ylabel('Total Fleet DOC [$M/yr]')
title('Entire Fleet Total Direct Operating Cost')
grid on

%% ==============================================================
%% Trade Study 4: Aspect Ratio vs Fleet DOC / Fuel / MTOM
%% ==============================================================
AspectRatios = 7:0.5:12;

fleet_DOC_AR  = zeros(size(AspectRatios));
fleet_fuel_AR = zeros(size(AspectRatios));
fleet_MTO_AR  = zeros(size(AspectRatios));

for i = 1:length(AspectRatios)
    ADPi = ADP0;
    ADPi.AR = AspectRatios(i);

    ADPi = B777.Size(ADPi);

    [BlockFuel, ~, ~, ~, ~] = B777.MissionAnalysis(ADPi, ADPi.TLAR.Range, ADPi.MTOM);

    [CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, ...
        MaintFixed, MaintVar, InsuranceCost, ~] = ...
        B777.Economics(ADPi.MTOM, ADPi.Span, BlockFuel, ...
                       ADPi.TLAR.FlightHours, ADPi.TLAR.ParkingDays, 'JetA1');

    fleet_total_cost_i = ADPi.Fleet_size * ( ...
        CrewCost + ParkingFee + MaintFixed + MaintVar + InsuranceCost + ...
        LandingFee * ADPi.TLAR.FlightsPerYear + ...
        FuelCost * ADPi.TLAR.FlightsPerYear );

    fleet_DOC_AR(i)  = fleet_total_cost_i;
    fleet_fuel_AR(i) = BlockFuel * ADPi.Fleet_size;
    fleet_MTO_AR(i)  = ADPi.MTOM;
end

figure(5); clf;
tt = tiledlayout(3,1);

nexttile;
plot(AspectRatios, fleet_DOC_AR/1e6, '-o', 'LineWidth', 1.5)
xlabel('Aspect Ratio')
ylabel('Fleet DOC [$M/yr]')
title('Aspect Ratio vs Direct Operating Cost')
grid on

nexttile;
plot(AspectRatios, fleet_fuel_AR/1e3, '-o', 'LineWidth', 1.5)
xlabel('Aspect Ratio')
ylabel('Fleet Block Fuel [t]')
title('Aspect Ratio vs Fuel Burn')
grid on

nexttile;
plot(AspectRatios, fleet_MTO_AR/1e3, '-o', 'LineWidth', 1.5)
xlabel('Aspect Ratio')
ylabel('MTOM [t]')
title('Aspect Ratio vs MTOM')
grid on

%% ==============================================================
%% Trade Study 5: MTOM vs Wing Loading
%% ==============================================================
WingLoadings = 7000:250:9000;   % [N/m^2]

mtoms_ws = zeros(size(WingLoadings));
final_ws = zeros(size(WingLoadings));

for i = 1:length(WingLoadings)
    ADPi = ADP0;

    % Set the trial wing loading
    ADPi.WingLoading = WingLoadings(i);

    % Optional: keep T/W fixed at baseline initial guess
    % ADPi.ThrustToWeightRatio = ADP0.ThrustToWeightRatio;

    % Re-size aircraft
    ADPi = B777.Size(ADPi);

    % Store outputs
    mtoms_ws(i) = ADPi.MTOM;
    final_ws(i) = ADPi.WingLoading;
end

figure(6); clf;
tt = tiledlayout(2,1);

nexttile;
plot(WingLoadings, mtoms_ws/1e3, '-o', 'LineWidth', 1.5)
xlabel('Input Wing Loading [N/m^2]')
ylabel('MTOM [t]')
title('MTOM vs Input Wing Loading')
grid on

nexttile;
plot(WingLoadings, final_ws, '-s', 'LineWidth', 1.5)
xlabel('Input Wing Loading [N/m^2]')
ylabel('Final Converged Wing Loading [N/m^2]')
title('Final Wing Loading vs Input Wing Loading')
grid on