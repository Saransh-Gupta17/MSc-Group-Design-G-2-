%% Sizing Tool

% Instantiate an instance of the aircraft class add define some initial
% parameters
ADP = B777.ADP();
ADP.TLAR = cast.TLAR.B777F(); % sets top level aircraft requirements

% ------------------------- set Hyper-parameters -------------------
ADP.TLAR.M_c = 0.84;
ADP.Fleet_size = 5;
ADP.TLAR.Payload = ADP.Total_Payload/ADP.Fleet_size;
ADP.c_ac = 8.5;
ADP.AR = 9.5;

%% --------------------- set Geometric parameters ---------------------
ADP.KinkPos = 10;       % spanwise position of TE kink in wing planform


[ADP.CabinLength,ADP.CabinRadius,ADP.L_total] = B777.geom.fuselage_sizer(ADP.Fleet_size, ADP.Pallet_size, ADP.CockpitLength, ADP.D_max);
%ADP.CabinRadius = 3.1;
%ADP.CabinLength = 63.7 - 6 - 3.1*2*1.48;
ADP.WingPos = 0.44*ADP.L_total;    % normalised wing position (% of fuselage length)
ADP.V_HT = 0.75;    % horizontal tail volume coefficent
ADP.V_VT = 0.07;    % vertical tail volume coefficent
ADP.HtpPos = 0.85*ADP.L_total;% normalised HTP position (% of fuselage length)
ADP.VtpPos = 0.82*ADP.L_total;% normalised VTP position (% of fuselage length)

% -------------------------- class-I estimates ---------------------------
% initial mission analysis to estimate MTOM
ADP.MTOM = 3.35*ADP.TLAR.Payload; % VERY basic guess of MTOM from payload

% initial estimate of fuel mass ( % of MTOM)
ADP.Mf_Fuel = 0.19; % maximum fuel mass
ADP.Mf_res = 0.03;  % reserve fuel mass

% initial estimate of mass fractions at important flight phases
ADP.Mf_Ldg = 0.68;  % maximum landing mass
ADP.Mf_TOC = 0.97;  % mass at teh Top of Climb (TOC)

%% -------------------------------- Sizing --------------------------------
% Note - see the "size" function at the bottum of this script
ADP = B777.Size(ADP);

% build the "Sized" geometry and plot it
[B7Geom,B7Mass] = B777.BuildGeometry(ADP); % get list of components geometries and masses

% plot the geometry (ontop of an image of a B777F for reference)
f = figure(1);
clf;
img = imread('B777F_planform.png'); 
imshow(img, 'XData', [0 63.7], 'YData', [-64.8 64.8]/2); 

cast.draw(B7Geom,B7Mass)
ax = gca;
ax.XAxis.Visible = "on";
ax.YAxis.Visible = "on";
axis equal
ylim([-0.5 0.5]*ADP.Span)


% print some key data points
d = B7Mass.GetData;
fprintf('MTOM: %0.0f t, Fuel Mass: %0.0f t, Wing Mass %0.0f t\n',ADP.MTOM/1e3,ADP.Mf_Fuel*ADP.MTOM/1e3,double(d(strcmp(d(:,1),"Wing"),2)));
fprintf('CD0: %0.3f, CD (CL=0.5): %0.3f \n',ADP.AeroPolar.CD(0),ADP.AeroPolar.CD(0.5));


%% Example of Costing Analysis
FuelType = 'JetA1'
[CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, MaintFixed, MaintVar, InsuranceCost, TotalCost] = B777.Economics(ADP.MTOM, ADP.Span, BlockFuel, ADP.TLAR.FlightHours, ADP.TLAR.ParkingDays, FuelType)
%% Example Trade study, comparing MTOM and Block Fuel as a function of wing span
% predefine spans to test
Spans = 50:5:100;

% per-allocate arrays for results
mtoms = Spans*0;
fuels = mtoms;

% loop over spans and size aircraft for each span
for i = 1:length(Spans)
    ADP.Span = Spans(i);
    ADP = B777.Size(ADP);
    mtoms(i) = ADP.MTOM;
    fuels(i) = ADP.Mf_Fuel*ADP.MTOM;
end

f = figure(2);
clf;
tt = tiledlayout(2,1);
nexttile(1);
plot(Spans,mtoms/1e3,'-s')
xlabel('Span [m]')
ylabel('MTOM [t]')

nexttile(2);
plot(Spans,fuels/1e3,'-o')
xlabel('Span [m]')
ylabel('Block Fuel [t]')

%% Example Trade study, comparing MTOM and Total Fleet Block Fuel as a function of fleet size
% predefine fleet sizes to test
FleetSizes = 6:1:14;

% save a clean baseline so each case starts from the same aircraft setup
ADP0 = ADP;

% pre-allocate arrays for results
mtoms_fleet = zeros(size(FleetSizes));              % MTOM per aircraft
blockfuel_per_aircraft = zeros(size(FleetSizes));   % block fuel per aircraft
total_blockfuel_fleet = zeros(size(FleetSizes));    % total block fuel for whole fleet

% loop over fleet sizes and size aircraft for each case
for i = 1:length(FleetSizes)
    % reset to baseline each iteration
    ADP = ADP0;
    
    % update fleet-dependent quantities
    ADP.Fleet_size = FleetSizes(i);
    ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;
    
    % recalculate fuselage sizing since payload arrangement changes
    [ADP.CabinLength, ADP.CabinRadius] = B777.geom.fuselage_sizer( ...
        ADP.Fleet_size, ADP.Pallet_size, ADP.CockpitLength, ADP.D_max);
    
    % resize aircraft
    ADP = B777.Size(ADP);
    
    % run mission analysis to get block fuel per aircraft
    [BlockFuel,~,~,~,~] = B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);
    
    % store results
    mtoms_fleet(i) = ADP.MTOM;
    blockfuel_per_aircraft(i) = BlockFuel;
    total_blockfuel_fleet(i) = ADP.Fleet_size * BlockFuel;
end

% plot results
f = figure(3);
clf;
tt = tiledlayout(3,1);

nexttile(1);
plot(FleetSizes, mtoms_fleet/1e3, '-s')
xlabel('Fleet Size')
ylabel('MTOM per Aircraft [t]')
grid on

nexttile(2);
plot(FleetSizes, blockfuel_per_aircraft/1e3, '-o')
xlabel('Fleet Size')
ylabel('Block Fuel per Aircraft [t]')
grid on

nexttile(3);
plot(FleetSizes, total_blockfuel_fleet/1e3, '-d')
xlabel('Fleet Size')
ylabel('Total Fleet Block Fuel [t]')
grid on

%% Example Trade study, comparing ENTIRE FLEET costs as a function of fleet size
% predefine fleet sizes to test
FleetSizes = 5:1:12;

% save a clean baseline so each case starts from the same aircraft setup
ADP0 = ADP;

% -------------------- operating assumptions --------------------
% Set these manually if they are not stored in ADP.TLAR
FlightHours = ADP.TLAR.FlightHours;      % annual flight hours per aircraft
ParkingDays = ADP.TLAR.ParkingDays;       % annual parking days per aircraft
FlightsPerYear = ADP.TLAR.FlightsPerYear;    % annual flights per aircraft
FuelType = 'JetA1';

% pre-allocate arrays for ENTIRE FLEET annual costs
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
    % reset to clean baseline
    ADP = ADP0;
    
    % update fleet-dependent quantities
    ADP.Fleet_size = FleetSizes(i);
    ADP.TLAR.Payload = ADP.Total_Payload / ADP.Fleet_size;
    
    % recalculate fuselage sizing
    [ADP.CabinLength, ADP.CabinRadius] = B777.geom.fuselage_sizer( ...
        ADP.Fleet_size, ADP.Pallet_size, ADP.CockpitLength, ADP.D_max);
    
    % resize aircraft
    ADP = B777.Size(ADP);
    
    % mission analysis gives block fuel per aircraft per mission
    [BlockFuel,~,~,~,~] = B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);
    
    % economics per aircraft
    [CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, ...
        MaintFixed, MaintVar, InsuranceCost, TotalCost_perAircraft] = ...
        B777.Economics(ADP.MTOM, ADP.Span, BlockFuel, FlightHours, ParkingDays, FuelType);
    
    % -------- convert to ENTIRE FLEET annual costs --------
    % annual per-aircraft costs:
    %   CrewCost, ParkingFee, MaintFixed, MaintVar, InsuranceCost
    % per-flight costs:
    %   LandingFee, FuelCost
    
    fleet_crew_cost(i)      = ADP.Fleet_size * CrewCost;
    fleet_landing_cost(i)   = ADP.Fleet_size * LandingFee * FlightsPerYear;
    fleet_parking_cost(i)   = ADP.Fleet_size * ParkingFee;
    fleet_fuel_cost(i)      = ADP.Fleet_size * FuelCost * FlightsPerYear;
    fleet_hull_value(i)     = ADP.Fleet_size * HullValue;
    fleet_maint_fixed(i)    = ADP.Fleet_size * MaintFixed;
    fleet_maint_var(i)      = ADP.Fleet_size * MaintVar;
    fleet_insurance_cost(i) = ADP.Fleet_size * InsuranceCost;
    
    fleet_total_cost(i) = fleet_crew_cost(i) + fleet_landing_cost(i) + ...
                          fleet_parking_cost(i) + fleet_fuel_cost(i) + ...
                          fleet_maint_fixed(i) + fleet_maint_var(i) + ...
                          fleet_insurance_cost(i);
end

%% Plot ENTIRE FLEET cost trade studies
f = figure(4);
clf;
tt = tiledlayout(4,2);

nexttile;
plot(FleetSizes, fleet_crew_cost/1e6, '-o')
xlabel('Fleet Size')
ylabel('Crew Cost [$M/yr]')
title('Entire Fleet Crew Cost')
grid on

nexttile;
plot(FleetSizes, fleet_landing_cost/1e6, '-o')
xlabel('Fleet Size')
ylabel('Landing Fees [$M/yr]')
title('Entire Fleet Landing Fees')
grid on

nexttile;
plot(FleetSizes, fleet_parking_cost/1e6, '-o')
xlabel('Fleet Size')
ylabel('Parking Fees [$M/yr]')
title('Entire Fleet Parking Fees')
grid on

nexttile;
plot(FleetSizes, fleet_fuel_cost/1e6, '-o')
xlabel('Fleet Size')
ylabel('Fuel Cost [$M/yr]')
title('Entire Fleet Fuel Cost')
grid on

nexttile;
plot(FleetSizes, fleet_hull_value/1e6, '-o')
xlabel('Fleet Size')
ylabel('Hull Value [$M]')
title('Entire Fleet Hull Value')
grid on

nexttile;
plot(FleetSizes, fleet_maint_fixed/1e6, '-o')
xlabel('Fleet Size')
ylabel('Fixed Maintenance [$M/yr]')
title('Entire Fleet Fixed Maintenance')
grid on

nexttile;
plot(FleetSizes, fleet_maint_var/1e6, '-o')
xlabel('Fleet Size')
ylabel('Variable Maintenance [$M/yr]')
title('Entire Fleet Variable Maintenance')
grid on

nexttile;
plot(FleetSizes, fleet_insurance_cost/1e6, '-o')
xlabel('Fleet Size')
ylabel('Insurance Cost [$M/yr]')
title('Entire Fleet Insurance')
grid on

f = figure(5);
clf;
plot(FleetSizes, fleet_total_cost/1e6, '-s','LineWidth',1.5)
xlabel('Fleet Size')
ylabel('Total Fleet DOC [$M/yr]')
title('Entire Fleet Total Direct Operating Cost')
grid on

%% Trade study: Aspect Ratio vs Direct Operating Cost (ENTIRE FLEET)

AspectRatios = 7:0.5:12;

ADP0 = ADP;

fleet_DOC_AR = zeros(size(AspectRatios));
fleet_fuel_AR = zeros(size(AspectRatios));
fleet_MTO_AR = zeros(size(AspectRatios));

for i = 1:length(AspectRatios)

    ADP = ADP0;

    % Update aspect ratio
    ADP.AR = AspectRatios(i);

    % Resize aircraft
    ADP = B777.Size(ADP);

    % Mission analysis
    [BlockFuel,~,~,~,~] = B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

    % Economics per aircraft
    [CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, ...
        MaintFixed, MaintVar, InsuranceCost, TotalCost_perAircraft] = ...
        B777.Economics(ADP.MTOM, ADP.Span, BlockFuel, ...
        ADP.TLAR.FlightHours, ADP.TLAR.ParkingDays, 'JetA1');

    % Convert to fleet annual cost
    fleet_total_cost = ADP.Fleet_size * ( ...
        CrewCost + ...
        ParkingFee + ...
        MaintFixed + ...
        MaintVar + ...
        InsuranceCost + ...
        LandingFee * ADP.TLAR.FlightsPerYear + ...
        FuelCost * ADP.TLAR.FlightsPerYear );

    fleet_DOC_AR(i) = fleet_total_cost;
    fleet_fuel_AR(i) = BlockFuel * ADP.Fleet_size;
    fleet_MTO_AR(i) = ADP.MTOM;

    figure(6)
clf
tiledlayout(3,1)

nexttile
plot(AspectRatios,fleet_DOC_AR/1e6,'-o','LineWidth',1.5)
xlabel('Aspect Ratio')
ylabel('Fleet DOC [$M/yr]')
title('Aspect Ratio vs Direct Operating Cost')
grid on

nexttile
plot(AspectRatios,fleet_fuel_AR/1e3,'-o')
xlabel('Aspect Ratio')
ylabel('Fleet Block Fuel [t]')
title('Aspect Ratio vs Fuel Burn')
grid on

nexttile
plot(AspectRatios,fleet_MTO_AR/1e3,'-o')
xlabel('Aspect Ratio')
ylabel('MTOM [t]')
title('Aspect Ratio vs MTOM')
grid on
end