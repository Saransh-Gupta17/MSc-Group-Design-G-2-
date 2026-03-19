function [CrewCost, LandingFee, ParkingFee, FuelCost, HullValue, MaintFixed, MaintVar, InsuranceCost, TotalCost] = Economics(MTOM, Span, BlockFuel_kg, FlightHours, ParkingDays, FuelType)
%DOC_COST_MODEL Estimates operating costs for an aircraft
%
% Inputs:
% MTOM         - Maximum take-off mass [kg]
% Span         - Wing span [m]
% BlockFuel_kg - Block fuel used per mission [kg]
% FlightHours  - Annual flight hours
% ParkingDays  - Number of parking days per year
% FuelType     - 'JetA1' or 'SAF'
%
% Outputs:
% CrewCost
% LandingFee
% ParkingFee
% FuelCost
% HullValue
% MaintFixed
% MaintVar
% InsuranceCost
% TotalCost

%% Crew Salaries
crew_salary = 150000;  % per crew member
num_crew = 4;
CrewCost = crew_salary * num_crew;

%% Landing Fees
landing_rate = 25; % $ per tonne
LandingFee = landing_rate * (MTOM/1000);

%% Parking Fees
if Span < 36
    parking_rate = 1000; % Code C
elseif Span < 52
    parking_rate = 2000; % Code D
elseif Span < 65
    parking_rate = 4000; % Code E
else
    parking_rate = 6000; % Code F
end

ParkingFee = parking_rate * ParkingDays;

%% Fuel Costs
fuel_density = 0.8; % kg/L
fuel_litres = BlockFuel_kg / fuel_density;

if strcmpi(FuelType,'JetA1')
    fuel_price = 1.00;
elseif strcmpi(FuelType,'SAF')
    fuel_price = 2.00;
else
    error('FuelType must be JetA1 or SAF')
end

FuelCost = fuel_litres * fuel_price;

%% Hull Value
HullValue = 44880 * MTOM^0.65;

%% Maintenance Costs
MaintFixed = 0.03 * HullValue;
MaintVar = 5e-6 * HullValue * FlightHours;

%% Insurance Costs
InsuranceCost = 0.005 * HullValue;

%% Total Cost
TotalCost = CrewCost + LandingFee + ParkingFee + FuelCost + MaintFixed + MaintVar + InsuranceCost;

end