function Econ = Economics(MTOM, Span, BlockFuel_kg, BlockTime_hr, ...
                          FuelType, NumLandings, NumCycles, ...
                          AnnualUtilisation_hr, AnnualFlights, ...
                          TotalInvestment, varargin)
% ECONOMICS  Class-I aircraft economics model aligned to lecture slides
%
% Returns COC and DOC breakdowns.
%
% Inputs
%   MTOM                [kg]
%   Span                [m]
%   BlockFuel_kg        fuel used per mission [kg]
%   BlockTime_hr        block time per mission [hr]
%   FuelType            'JetA1' or 'SAF'
%   NumLandings         number of landings per mission
%   NumCycles           number of cycles per mission (normally 1)
%   AnnualUtilisation_hr annual flight hours [hr/year]
%   AnnualFlights       flights per year [1/year]
%   TotalInvestment     aircraft acquisition / total investment [$]
%
% Optional name-value inputs
%   'CockpitCrewPerYear'   default 300000
%   'CabinCrewPerYear'     default 300000
%   'ParkingDaysPerYear'   default 0
%   'NavChargePerFlight'   default 0
%   'UseImprovedMaint'     default false
%   'EngineCost'           default 0
%   'NumEngines'           default 2
%
% Output
%   Econ struct with detailed breakdown:
%       Econ.COC_trip
%       Econ.DOC_trip
%       Econ.COC_annual
%       Econ.DOC_annual
%       Econ.breakdown

    %% ---------------- Defaults ----------------
    p = inputParser;
    addParameter(p, 'CockpitCrewPerYear', 300000);  % 2 pilots + relief assumption lumped
    addParameter(p, 'CabinCrewPerYear',   300000);  % simple placeholder split
    addParameter(p, 'ParkingDaysPerYear', 365);
    addParameter(p, 'NavChargePerFlight', 0);
    addParameter(p, 'UseImprovedMaint', false);
    addParameter(p, 'EngineCost', 0);
    addParameter(p, 'NumEngines', 2);
    parse(p, varargin{:});
    opt = p.Results;

    %% ---------------- Crew ----------------
    CockpitCrewAnnual = opt.CockpitCrewPerYear;
    CabinCrewAnnual   = opt.CabinCrewPerYear;
    CrewAnnual        = CockpitCrewAnnual + CabinCrewAnnual;

    CockpitCrewTrip = CockpitCrewAnnual / max(AnnualFlights,1);
    CabinCrewTrip   = CabinCrewAnnual   / max(AnnualFlights,1);

    %% ---------------- Landing fees ----------------
    landing_rate = 25; % $/tonne
    LandingFeeTrip = landing_rate * (MTOM/1000) * NumLandings;

    %% ---------------- Parking fees ----------------
    if Span < 24
        parking_rate = 0;      % outside stated code range
    elseif Span < 36
        parking_rate = 1000;   % Code C
    elseif Span < 52
        parking_rate = 2000;   % Code D
    elseif Span < 65
        parking_rate = 4000;   % Code E
    else
        parking_rate = 6000;   % Code F
    end

    ParkingAnnual = parking_rate * opt.ParkingDaysPerYear;
    ParkingTrip   = ParkingAnnual / max(AnnualFlights,1);

    %% ---------------- Fuel ----------------
    fuel_density = 0.8; % kg/L
    fuel_litres = BlockFuel_kg / fuel_density;

    switch lower(FuelType)
        case 'jeta1'
            fuel_price = 1.00;   % $/L
        case 'saf'
            fuel_price = 2.00;   % $/L
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
    % Simple class-I model
    MaintFixedAnnual_simple = 0.03 * HullValue;
    MaintVarPerFH_simple    = 5e-6 * HullValue;
    MaintVarAnnual_simple   = MaintVarPerFH_simple * AnnualUtilisation_hr;

    % Improved maintenance model from slides:
    % material cost/FH = 3.3*(Ca/1e6) + 14.2 + [58*(Ce/1e6) - 26.1]*Ne
    % material cost/cycle = 4.0*(Ca/1e6) + 9.3 + [7.5*(Ce/1e6) + 5.6]*Ne
    if opt.UseImprovedMaint
        Ce = opt.EngineCost;
        Ne = opt.NumEngines;
        Ca = max(TotalInvestment - Ne*Ce, 0);  % aircraft cost less engines

        MaintMatPerFH    = 3.3*(Ca/1e6) + 14.2 + (58*(Ce/1e6) - 26.1)*Ne;
        MaintMatPerCycle = 4.0*(Ca/1e6) + 9.3  + (7.5*(Ce/1e6) + 5.6)*Ne;

        MaintAnnual = MaintMatPerFH * AnnualUtilisation_hr + ...
                      MaintMatPerCycle * AnnualFlights;
        MaintTrip = MaintMatPerFH * BlockTime_hr + ...
                    MaintMatPerCycle * NumCycles;

        MaintFixedAnnual = 0;
        MaintVarAnnual   = MaintAnnual;
    else
        MaintFixedAnnual = MaintFixedAnnual_simple;
        MaintVarAnnual   = MaintVarAnnual_simple;
        MaintTrip = MaintFixedAnnual / max(AnnualFlights,1) + ...
                    MaintVarPerFH_simple * BlockTime_hr;
    end

    %% ---------------- Financial costs (DOC only) ----------------
    % From slides:
    % Depreciation = Total Investment / (14 * Utilisation)
    % Interest     = 0.05 * Total Investment
    % Insurance    = 0.006 * Hull Value

    % Here Utilisation is interpreted as annual flights for trip conversion,
    % and annual ownership costs for yearly totals.
    DepreciationAnnual = TotalInvestment / 14;
    InterestAnnual     = 0.05 * TotalInvestment;
    InsuranceAnnual    = 0.006 * HullValue;

    DepreciationTrip = DepreciationAnnual / max(AnnualFlights,1);
    InterestTrip     = InterestAnnual     / max(AnnualFlights,1);
    InsuranceTrip    = InsuranceAnnual    / max(AnnualFlights,1);

    %% ---------------- COC and DOC ----------------
    COC_trip = FuelTrip + LandingFeeTrip + CockpitCrewTrip + CabinCrewTrip + ...
               NavTrip + MaintTrip + ParkingTrip;

    DOC_trip = COC_trip + DepreciationTrip + InterestTrip + InsuranceTrip;

    COC_annual = FuelAnnual + LandingFeeTrip*AnnualFlights + CockpitCrewAnnual + ...
                 CabinCrewAnnual + NavAnnual + ParkingAnnual + ...
                 MaintFixedAnnual + MaintVarAnnual;

    DOC_annual = COC_annual + DepreciationAnnual + InterestAnnual + InsuranceAnnual;

    %% ---------------- Output struct ----------------
    Econ = struct();

    Econ.COC_trip   = COC_trip;
    Econ.DOC_trip   = DOC_trip;
    Econ.COC_annual = COC_annual;
    Econ.DOC_annual = DOC_annual;

    Econ.breakdown = struct( ...
        'FuelTrip', FuelTrip, ...
        'LandingFeeTrip', LandingFeeTrip, ...
        'CockpitCrewTrip', CockpitCrewTrip, ...
        'CabinCrewTrip', CabinCrewTrip, ...
        'NavigationTrip', NavTrip, ...
        'ParkingTrip', ParkingTrip, ...
        'MaintTrip', MaintTrip, ...
        'DepreciationTrip', DepreciationTrip, ...
        'InterestTrip', InterestTrip, ...
        'InsuranceTrip', InsuranceTrip, ...
        'HullValue', HullValue, ...
        'MaintFixedAnnual', MaintFixedAnnual, ...
        'MaintVarAnnual', MaintVarAnnual, ...
        'DepreciationAnnual', DepreciationAnnual, ...
        'InterestAnnual', InterestAnnual, ...
        'InsuranceAnnual', InsuranceAnnual);
end