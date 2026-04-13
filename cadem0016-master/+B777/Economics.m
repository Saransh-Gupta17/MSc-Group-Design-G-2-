function Econ = Economics(MTOM, Span, BlockFuel_kg, BlockTime_hr, ...
                          FuelType, NumLandings, NumCycles, ...
                          AnnualUtilisation_hr, AnnualFlights, ...
                          TotalInvestment, varargin)
% ECONOMICS  Class-I aircraft economics model with fleet lifecycle cost
%
% Returns COC and DOC breakdowns, plus fleet lifecycle cost over a chosen
% number of years (default 20 years).

    %% ---------------- Defaults ----------------
    p = inputParser;
    addParameter(p, 'CockpitCrewPerYear', 300000);
    addParameter(p, 'CabinCrewPerYear',   300000);
    addParameter(p, 'ParkingDaysPerYear', 365);
    addParameter(p, 'NavChargePerFlight', 0);
    addParameter(p, 'UseImprovedMaint', false);
    addParameter(p, 'EngineCost', 0);
    addParameter(p, 'NumEngines', 2);
    addParameter(p, 'FleetSize', 1);
    addParameter(p, 'LifecycleYears', 20);
    parse(p, varargin{:});
    opt = p.Results;

    FleetSize      = opt.FleetSize;
    LifecycleYears = opt.LifecycleYears;

    %% ---------------- Crew ----------------
    CockpitCrewAnnual = opt.CockpitCrewPerYear;
    CabinCrewAnnual   = opt.CabinCrewPerYear;
    CrewAnnual        = CockpitCrewAnnual + CabinCrewAnnual;

    CockpitCrewTrip = CockpitCrewAnnual / max(AnnualFlights, 1);
    CabinCrewTrip   = CabinCrewAnnual   / max(AnnualFlights, 1);

    %% ---------------- Landing fees ----------------
    landing_rate = 25; % $/tonne
    LandingFeeTrip = landing_rate * (MTOM / 1000) * NumLandings;
    LandingAnnual  = LandingFeeTrip * AnnualFlights;

    %% ---------------- Parking fees ----------------
    if Span < 24
        parking_rate = 0;
    elseif Span < 36
        parking_rate = 1000;
    elseif Span < 52
        parking_rate = 2000;
    elseif Span < 65
        parking_rate = 4000;
    else
        parking_rate = 6000;
    end

    ParkingAnnual = parking_rate * opt.ParkingDaysPerYear;
    ParkingTrip   = ParkingAnnual / max(AnnualFlights, 1);

    %% ---------------- Fuel ----------------
    fuel_density = 0.8; % kg/L
    fuel_litres  = BlockFuel_kg / fuel_density;

    switch lower(FuelType)
        case 'jeta1'
            fuel_price = 1.00;
        case 'saf'
            fuel_price = 2.00;
        otherwise
            error('FuelType must be ''JetA1'' or ''SAF''.');
    end

    FuelTrip   = fuel_litres * fuel_price;
    FuelAnnual = FuelTrip * AnnualFlights;

    %% ---------------- Hull value ----------------
    HullValue = 44880 * MTOM^0.65;

    %% ---------------- Navigation charges ----------------
    NavTrip   = opt.NavChargePerFlight;
    NavAnnual = NavTrip * AnnualFlights;

    %% ---------------- Maintenance ----------------
    MaintFixedAnnual_simple = 0.03 * HullValue;
    MaintVarPerFH_simple    = 5e-6 * HullValue;
    MaintVarAnnual_simple   = MaintVarPerFH_simple * AnnualUtilisation_hr;

    if opt.UseImprovedMaint
        Ce = opt.EngineCost;
        Ne = opt.NumEngines;
        Ca = max(TotalInvestment - Ne * Ce, 0);

        MaintMatPerFH    = 3.3 * (Ca / 1e6) + 14.2 + (58 * (Ce / 1e6) - 26.1) * Ne;
        MaintMatPerCycle = 4.0 * (Ca / 1e6) + 9.3  + (7.5 * (Ce / 1e6) + 5.6) * Ne;

        MaintAnnual = MaintMatPerFH * AnnualUtilisation_hr + ...
                      MaintMatPerCycle * AnnualFlights;

        MaintTrip = MaintMatPerFH * BlockTime_hr + ...
                    MaintMatPerCycle * NumCycles;

        MaintFixedAnnual = 0;
        MaintVarAnnual   = MaintAnnual;
    else
        MaintFixedAnnual = MaintFixedAnnual_simple;
        MaintVarAnnual   = MaintVarAnnual_simple;

        MaintTrip = MaintFixedAnnual / max(AnnualFlights, 1) + ...
                    MaintVarPerFH_simple * BlockTime_hr;

        MaintAnnual = MaintFixedAnnual + MaintVarAnnual;
    end

    %% ---------------- Financial / ownership terms ----------------
    DepreciationAnnual = TotalInvestment / 14;
    InterestAnnual     = 0.05 * TotalInvestment;
    InsuranceAnnual    = 0.006 * HullValue;

    DepreciationTrip = DepreciationAnnual / max(AnnualFlights, 1);
    InterestTrip     = InterestAnnual     / max(AnnualFlights, 1);
    InsuranceTrip    = InsuranceAnnual    / max(AnnualFlights, 1);

    %% ---------------- COC and DOC ----------------
    COC_trip = FuelTrip + LandingFeeTrip + CockpitCrewTrip + CabinCrewTrip + ...
               NavTrip + MaintTrip + ParkingTrip;

    DOC_trip = COC_trip + DepreciationTrip + InterestTrip + InsuranceTrip;

    COC_annual = FuelAnnual + LandingAnnual + CockpitCrewAnnual + ...
                 CabinCrewAnnual + NavAnnual + ParkingAnnual + ...
                 MaintFixedAnnual + MaintVarAnnual;

    DOC_annual = COC_annual + DepreciationAnnual + InterestAnnual + InsuranceAnnual;

    %% ---------------- Fleet annual costs ----------------
    FleetCapitalCost   = FleetSize * TotalInvestment;

    FleetCOC_annual    = FleetSize * COC_annual;
    FleetDOC_annual    = FleetSize * DOC_annual;

    FleetFuelAnnual      = FleetSize * FuelAnnual;
    FleetLandingAnnual   = FleetSize * LandingAnnual;
    FleetCrewAnnual      = FleetSize * CrewAnnual;
    FleetNavAnnual       = FleetSize * NavAnnual;
    FleetParkingAnnual   = FleetSize * ParkingAnnual;
    FleetMaintAnnual     = FleetSize * MaintAnnual;
    FleetInsuranceAnnual = FleetSize * InsuranceAnnual;

    %% ---------------- Lifecycle costs ----------------
    FleetCOC_20yr = LifecycleYears * FleetCOC_annual;
    FleetDOC_20yr = LifecycleYears * FleetDOC_annual;

    FleetLifecycleCostCOC = FleetCapitalCost + FleetCOC_20yr;
    FleetLifecycleCostDOC = FleetCapitalCost + FleetDOC_20yr;

    %% ---------------- Output struct ----------------
    Econ = struct();

    Econ.COC_trip = COC_trip;
    Econ.DOC_trip = DOC_trip;
    Econ.COC_annual = COC_annual;
    Econ.DOC_annual = DOC_annual;

    Econ.FleetSize      = FleetSize;
    Econ.LifecycleYears = LifecycleYears;

    Econ.FleetCapitalCost = FleetCapitalCost;
    Econ.FleetCOC_annual  = FleetCOC_annual;
    Econ.FleetDOC_annual  = FleetDOC_annual;

    Econ.FleetCOC_20yr         = FleetCOC_20yr;
    Econ.FleetDOC_20yr         = FleetDOC_20yr;
    Econ.FleetLifecycleCostCOC = FleetLifecycleCostCOC;
    Econ.FleetLifecycleCostDOC = FleetLifecycleCostDOC;

    Econ.Objective = FleetLifecycleCostCOC;

    Econ.breakdown = struct( ...
        'FuelTrip', FuelTrip, ...
        'FuelAnnual', FuelAnnual, ...
        'LandingFeeTrip', LandingFeeTrip, ...
        'LandingAnnual', LandingAnnual, ...
        'CockpitCrewTrip', CockpitCrewTrip, ...
        'CabinCrewTrip', CabinCrewTrip, ...
        'CockpitCrewAnnual', CockpitCrewAnnual, ...
        'CabinCrewAnnual', CabinCrewAnnual, ...
        'CrewAnnual', CrewAnnual, ...
        'NavigationTrip', NavTrip, ...
        'NavigationAnnual', NavAnnual, ...
        'ParkingTrip', ParkingTrip, ...
        'ParkingAnnual', ParkingAnnual, ...
        'MaintTrip', MaintTrip, ...
        'MaintFixedAnnual', MaintFixedAnnual, ...
        'MaintVarAnnual', MaintVarAnnual, ...
        'MaintAnnual', MaintAnnual, ...
        'DepreciationTrip', DepreciationTrip, ...
        'InterestTrip', InterestTrip, ...
        'InsuranceTrip', InsuranceTrip, ...
        'DepreciationAnnual', DepreciationAnnual, ...
        'InterestAnnual', InterestAnnual, ...
        'InsuranceAnnual', InsuranceAnnual, ...
        'HullValue', HullValue, ...
        'FleetFuelAnnual', FleetFuelAnnual, ...
        'FleetLandingAnnual', FleetLandingAnnual, ...
        'FleetCrewAnnual', FleetCrewAnnual, ...
        'FleetNavAnnual', FleetNavAnnual, ...
        'FleetParkingAnnual', FleetParkingAnnual, ...
        'FleetMaintAnnual', FleetMaintAnnual, ...
        'FleetInsuranceAnnual', FleetInsuranceAnnual);
end