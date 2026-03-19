function [BlockFuel, TripFuel, ResFuel, Mf_TOC, ...
          MissionTime, Mission] = ...
          MissionAnalysis(ADP, tripRange, M_TO)
%MISSIONANALYSIS Conduct mission analysis using nominal mission profile

arguments
    ADP
    tripRange
    M_TO = ADP.MTOM
end

%% ----------------------------- Constants ------------------------------
g = 9.81;
rho0 = 1.225; %#ok<NASGU>

%% ---------------------- Mission assumptions ---------------------------
TaxiTime_out   = 20 * 60;      % s
TaxiTime_in    = 20 * 60;      % s
TakeoffTime    = 1 * 60;       % s
ClimbTime      = 30 * 60;      % s
ApproachTime   = 10 * 60;      % s
LoiterTime     = 30 * 60;      % s
ContTime       = 5 * 60;       % s
AltRange       = 350e3;        % m

gamma_to      = 10 * pi / 180;                 % rad
ROC_to        = (20000 - 1500) / (30 * 196.85);
ROC_climb_1   = (20000 - 1500) / (30 * 196.85);
ROC_climb_2   = ROC_climb_1;
ROC_climb_3   = ROC_climb_1;
ROD_descent_1 = ROC_climb_1;
ROD_descent_2 = ROD_descent_1;
ROD_descent_3 = ROD_descent_1;

V_climb_1   = 250 * 0.514444;   % m/s
V_climb_2   = 300 * 0.514444;   % m/s
V_descent_2 = V_climb_2;
V_descent_3 = V_climb_1;

IdleFrac     = 0.07;
TakeoffFrac  = 1.00; %#ok<NASGU>
ClimbFrac    = 0.85;
DescentFrac  = 0.12;
ApproachFrac = 0.18;

M_cruise = ADP.TLAR.M_c;
M_to     = 0.25;
M_ds     = 0.20;
M_climb  = min(M_cruise, 0.78);
M_desc   = min(M_cruise, 0.70);

Mission = struct();

%% -------------------- Initial mass bookkeeping ------------------------
m_current = M_TO;

%% ============================================================
%% 2) Taxi out
%% ============================================================
FuelTaxiOut = segmentFuelFromThrustFrac(ADP, m_current, 0.0, 0, IdleFrac, TaxiTime_out);
m_current = m_current - FuelTaxiOut;

Mission.TaxiOut.Fuel = FuelTaxiOut;
Mission.TaxiOut.Time = TaxiTime_out;

%% ============================================================
%% 3) Take-off / initial climb / acceleration
%% ============================================================
[rho_to, a_to, ~, ~] = cast.atmos(0);
V_to = M_to * a_to;
W = m_current * g;

CL_to = W * cos(gamma_to) / (0.5 * rho_to * V_to^2 * ADP.WingArea);
CD_to = ADP.AeroPolar.CD(CL_to);
D_to  = 0.5 * rho_to * V_to^2 * ADP.WingArea * CD_to;

T_req_to = D_to + W * ROC_to / V_to;
FuelTakeoff = segmentFuelFromThrust(ADP, M_to, 0, T_req_to, TakeoffTime);

m_current = m_current - FuelTakeoff;

Mission.Takeoff.Fuel = FuelTakeoff;
Mission.Takeoff.Time = TakeoffTime;
Mission.Takeoff.CL = CL_to;
Mission.Takeoff.CD = CD_to;
Mission.Takeoff.Drag = D_to;
Mission.Takeoff.Treq = T_req_to;
Mission.Takeoff.V = V_to;

%% ============================================================
%% 4) Climb 1: 1500 ft to 10000 ft in 500 ft steps
%% ============================================================
h_start = 1500 / SI.ft;
h_end   = 10000 / SI.ft;
dh      = 500 / SI.ft;

nSteps = ceil((h_end - h_start) / dh);

FuelClimb1 = 0;
TimeClimb1 = 0;

Mission.TakeoffClimb.StepFuel = zeros(nSteps,1);
Mission.TakeoffClimb.StepMass = zeros(nSteps,1);
Mission.TakeoffClimb.StepAlt  = zeros(nSteps,1);
Mission.TakeoffClimb.StepMach = zeros(nSteps,1);
Mission.TakeoffClimb.StepCL   = zeros(nSteps,1);
Mission.TakeoffClimb.StepCD   = zeros(nSteps,1);
Mission.TakeoffClimb.StepDrag = zeros(nSteps,1);
Mission.TakeoffClimb.StepTreq = zeros(nSteps,1);
Mission.TakeoffClimb.StepTime = zeros(nSteps,1);
Mission.TakeoffClimb.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_low = h_start + (i-1) * dh;
    h_high = min(h_low + dh, h_end);
    h_mid = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = V_climb_1 / a;
    W = m_current * g;

    CL_climb_1 = W / (0.5 * rho * V_climb_1^2 * ADP.WingArea);
    CD_climb_1 = ADP.AeroPolar.CD(CL_climb_1);
    D_climb_1  = 0.5 * rho * V_climb_1^2 * ADP.WingArea * CD_climb_1;

    T_req_climb_1 = D_climb_1 + W * ROC_climb_1 / V_climb_1;

    dh_step = h_high - h_low;
    dt = dh_step / ROC_climb_1;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_climb_1 * dt;

    m_current = m_current - FuelStep;
    FuelClimb1 = FuelClimb1 + FuelStep;
    TimeClimb1 = TimeClimb1 + dt;

    Mission.TakeoffClimb.StepFuel(i) = FuelStep;
    Mission.TakeoffClimb.StepMass(i) = m_current;
    Mission.TakeoffClimb.StepAlt(i)  = h_mid;
    Mission.TakeoffClimb.StepMach(i) = Mach;
    Mission.TakeoffClimb.StepCL(i)   = CL_climb_1;
    Mission.TakeoffClimb.StepCD(i)   = CD_climb_1;
    Mission.TakeoffClimb.StepDrag(i) = D_climb_1;
    Mission.TakeoffClimb.StepTreq(i) = T_req_climb_1;
    Mission.TakeoffClimb.StepTime(i) = dt;
    Mission.TakeoffClimb.StepV(i)    = V_climb_1;
end

Mission.TakeoffClimb.Fuel = FuelClimb1;
Mission.TakeoffClimb.Time = TimeClimb1;

%% ============================================================
%% 5) Climb 2: 10000 ft to 20000 ft in 500 ft steps
%% ============================================================
h_start = 10000 / SI.ft;
h_end   = 20000 / SI.ft;
dh      = 500 / SI.ft;

nSteps = ceil((h_end - h_start) / dh);

FuelClimb2 = 0;
TimeClimb2 = 0;

Mission.Climb2.StepFuel = zeros(nSteps,1);
Mission.Climb2.StepMass = zeros(nSteps,1);
Mission.Climb2.StepAlt  = zeros(nSteps,1);
Mission.Climb2.StepMach = zeros(nSteps,1);
Mission.Climb2.StepCL   = zeros(nSteps,1);
Mission.Climb2.StepCD   = zeros(nSteps,1);
Mission.Climb2.StepDrag = zeros(nSteps,1);
Mission.Climb2.StepTreq = zeros(nSteps,1);
Mission.Climb2.StepTime = zeros(nSteps,1);
Mission.Climb2.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_low = h_start + (i-1) * dh;
    h_high = min(h_low + dh, h_end);
    h_mid = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = V_climb_2 / a;
    W = m_current * g;

    CL_climb_2 = W / (0.5 * rho * V_climb_2^2 * ADP.WingArea);
    CD_climb_2 = ADP.AeroPolar.CD(CL_climb_2);
    D_climb_2  = 0.5 * rho * V_climb_2^2 * ADP.WingArea * CD_climb_2;

    T_req_climb_2 = D_climb_2 + W * ROC_climb_2 / V_climb_2;

    dh_step = h_high - h_low;
    dt = dh_step / ROC_climb_2;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_climb_2 * dt;

    m_current = m_current - FuelStep;
    FuelClimb2 = FuelClimb2 + FuelStep;
    TimeClimb2 = TimeClimb2 + dt;

    Mission.Climb2.StepFuel(i) = FuelStep;
    Mission.Climb2.StepMass(i) = m_current;
    Mission.Climb2.StepAlt(i)  = h_mid;
    Mission.Climb2.StepMach(i) = Mach;
    Mission.Climb2.StepCL(i)   = CL_climb_2;
    Mission.Climb2.StepCD(i)   = CD_climb_2;
    Mission.Climb2.StepDrag(i) = D_climb_2;
    Mission.Climb2.StepTreq(i) = T_req_climb_2;
    Mission.Climb2.StepTime(i) = dt;
    Mission.Climb2.StepV(i)    = V_climb_2;
end

Mission.Climb2.Fuel = FuelClimb2;
Mission.Climb2.Time = TimeClimb2;

%% ============================================================
%% 6) Climb 3: 20000 ft to cruise altitude in 500 ft steps
%% ============================================================
h_start = 20000 / SI.ft;
h_end   = ADP.cruise_altitude;
dh      = 500 / SI.ft;

nSteps = max(0, ceil((h_end - h_start) / dh));

FuelClimb3 = 0;
TimeClimb3 = 0;

Mission.Climb3.StepFuel = zeros(nSteps,1);
Mission.Climb3.StepMass = zeros(nSteps,1);
Mission.Climb3.StepAlt  = zeros(nSteps,1);
Mission.Climb3.StepMach = zeros(nSteps,1);
Mission.Climb3.StepCL   = zeros(nSteps,1);
Mission.Climb3.StepCD   = zeros(nSteps,1);
Mission.Climb3.StepDrag = zeros(nSteps,1);
Mission.Climb3.StepTreq = zeros(nSteps,1);
Mission.Climb3.StepTime = zeros(nSteps,1);
Mission.Climb3.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_low = h_start + (i-1) * dh;
    h_high = min(h_low + dh, h_end);
    h_mid = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = min(ADP.TLAR.M_c, 0.80);
    V_climb_3 = Mach * a;
    W = m_current * g;

    CL_climb_3 = W / (0.5 * rho * V_climb_3^2 * ADP.WingArea);
    CD_climb_3 = ADP.AeroPolar.CD(CL_climb_3);
    D_climb_3  = 0.5 * rho * V_climb_3^2 * ADP.WingArea * CD_climb_3;

    T_req_climb_3 = D_climb_3 + W * ROC_climb_3 / V_climb_3;

    dh_step = h_high - h_low;
    dt = dh_step / ROC_climb_3;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_climb_3 * dt;

    m_current = m_current - FuelStep;
    FuelClimb3 = FuelClimb3 + FuelStep;
    TimeClimb3 = TimeClimb3 + dt;

    Mission.Climb3.StepFuel(i) = FuelStep;
    Mission.Climb3.StepMass(i) = m_current;
    Mission.Climb3.StepAlt(i)  = h_mid;
    Mission.Climb3.StepMach(i) = Mach;
    Mission.Climb3.StepCL(i)   = CL_climb_3;
    Mission.Climb3.StepCD(i)   = CD_climb_3;
    Mission.Climb3.StepDrag(i) = D_climb_3;
    Mission.Climb3.StepTreq(i) = T_req_climb_3;
    Mission.Climb3.StepTime(i) = dt;
    Mission.Climb3.StepV(i)    = V_climb_3;
end

Mission.Climb3.Fuel = FuelClimb3;
Mission.Climb3.Time = TimeClimb3;

FuelClimb = FuelClimb1 + FuelClimb2 + FuelClimb3;
TimeClimb = TimeClimb1 + TimeClimb2 + TimeClimb3;

Mf_TOC = m_current / M_TO;

%% ============================================================
%% 7) Cruise: stepped cruise-climb with constant CL target
%% ============================================================
RangeClimb1 = sum(Mission.TakeoffClimb.StepTime .* Mission.TakeoffClimb.StepV);
RangeClimb2 = sum(Mission.Climb2.StepTime .* Mission.Climb2.StepV);
RangeClimb3 = sum(Mission.Climb3.StepTime .* Mission.Climb3.StepV);

RangeClimbTotal = RangeClimb1 + RangeClimb2 + RangeClimb3;

RangeCruise = tripRange - 2 * RangeClimbTotal;
RangeCruise = max(RangeCruise, 0);

dR = 50e3;
nSteps = max(1, ceil(max(RangeCruise, 1) / dR));

FuelCruise = 0;
TimeCruise = 0;
RangeCruiseActual = 0;

alt_cruise_step = ADP.cruise_altitude;

[rho_c0, a_c0, ~, ~] = cast.atmos(alt_cruise_step);
V_c0 = M_cruise * a_c0;
W_c0 = m_current * g;

CL_target = W_c0 / (0.5 * rho_c0 * V_c0^2 * ADP.WingArea);

Mission.Cruise.StepFuel  = zeros(nSteps,1);
Mission.Cruise.StepMass  = zeros(nSteps,1);
Mission.Cruise.StepAlt   = zeros(nSteps,1);
Mission.Cruise.StepMach  = zeros(nSteps,1);
Mission.Cruise.StepCL    = zeros(nSteps,1);
Mission.Cruise.StepCD    = zeros(nSteps,1);
Mission.Cruise.StepLD    = zeros(nSteps,1);
Mission.Cruise.StepDrag  = zeros(nSteps,1);
Mission.Cruise.StepTreq  = zeros(nSteps,1);
Mission.Cruise.StepTime  = zeros(nSteps,1);
Mission.Cruise.StepRange = zeros(nSteps,1);
Mission.Cruise.StepV     = zeros(nSteps,1);

for i = 1:nSteps
    dR_step = min(dR, RangeCruise - RangeCruiseActual);
    if dR_step <= 0
        break
    end

    [rho, a, ~, ~] = cast.atmos(alt_cruise_step);

    V = M_cruise * a;
    W = m_current * g;

    CL = W / (0.5 * rho * V^2 * ADP.WingArea);
    CD = ADP.AeroPolar.CD(CL);
    D  = 0.5 * rho * V^2 * ADP.WingArea * CD;
    LD = CL / CD;

    T_req = D;
    dt = dR_step / V;

    TSFC = ADP.Engine.TSFC(M_cruise, alt_cruise_step);
    FuelStep = TSFC * T_req * dt;

    m_current = m_current - FuelStep;
    FuelCruise = FuelCruise + FuelStep;
    TimeCruise = TimeCruise + dt;
    RangeCruiseActual = RangeCruiseActual + dR_step;

    Mission.Cruise.StepFuel(i)  = FuelStep;
    Mission.Cruise.StepMass(i)  = m_current;
    Mission.Cruise.StepAlt(i)   = alt_cruise_step;
    Mission.Cruise.StepMach(i)  = M_cruise;
    Mission.Cruise.StepCL(i)    = CL;
    Mission.Cruise.StepCD(i)    = CD;
    Mission.Cruise.StepLD(i)    = LD;
    Mission.Cruise.StepDrag(i)  = D;
    Mission.Cruise.StepTreq(i)  = T_req;
    Mission.Cruise.StepTime(i)  = dt;
    Mission.Cruise.StepRange(i) = dR_step;
    Mission.Cruise.StepV(i)     = V;

    alt_grid = linspace(ADP.cruise_altitude, 43e3 / SI.ft, 200);
    [rho_grid, a_grid, ~, ~] = cast.atmos(alt_grid);

    CL_grid = m_current * g ./ ...
        (0.5 .* rho_grid .* (M_cruise .* a_grid).^2 .* ADP.WingArea);

    [~, idxAlt] = min(abs(CL_grid - CL_target));
    alt_cruise_step = alt_grid(idxAlt);
end

Mission.Cruise.Fuel = FuelCruise;
Mission.Cruise.Time = TimeCruise;
Mission.Cruise.Range = RangeCruiseActual;
Mission.Cruise.Altitude = ADP.cruise_altitude;
Mission.Cruise.FL = round(ADP.cruise_altitude * SI.ft / 100, 0);
Mission.Cruise.CL = CL_target;
Mission.Cruise.CD = ADP.AeroPolar.CD(CL_target);
Mission.Cruise.LD = CL_target / ADP.AeroPolar.CD(CL_target);

%% ============================================================
%% 8) Descent 1: cruise altitude to 20000 ft in 500 ft steps
%% ============================================================
h_start = ADP.cruise_altitude;
h_end   = 20000 / SI.ft;
dh      = 500 / SI.ft;

nSteps = max(0, ceil((h_start - h_end) / dh));

FuelDescent1 = 0;
TimeDescent1 = 0;

Mission.Descent1.StepFuel = zeros(nSteps,1);
Mission.Descent1.StepMass = zeros(nSteps,1);
Mission.Descent1.StepAlt  = zeros(nSteps,1);
Mission.Descent1.StepMach = zeros(nSteps,1);
Mission.Descent1.StepCL   = zeros(nSteps,1);
Mission.Descent1.StepCD   = zeros(nSteps,1);
Mission.Descent1.StepDrag = zeros(nSteps,1);
Mission.Descent1.StepTreq = zeros(nSteps,1);
Mission.Descent1.StepTime = zeros(nSteps,1);
Mission.Descent1.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_high = h_start - (i-1) * dh;
    h_low  = max(h_high - dh, h_end);
    h_mid  = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = min(ADP.TLAR.M_c, 0.80);
    V_descent_1 = Mach * a;
    W = m_current * g;

    CL_descent_1 = W / (0.5 * rho * V_descent_1^2 * ADP.WingArea);
    CD_descent_1 = ADP.AeroPolar.CD(CL_descent_1);
    D_descent_1  = 0.5 * rho * V_descent_1^2 * ADP.WingArea * CD_descent_1;

    T_idle = IdleFrac * 0.3 * m_current * g;
    T_req_descent_1 = max(T_idle, D_descent_1 - W * ROD_descent_1 / V_descent_1);

    dh_step = h_high - h_low;
    dt = dh_step / ROD_descent_1;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_descent_1 * dt;

    m_current = m_current - FuelStep;
    FuelDescent1 = FuelDescent1 + FuelStep;
    TimeDescent1 = TimeDescent1 + dt;

    Mission.Descent1.StepFuel(i) = FuelStep;
    Mission.Descent1.StepMass(i) = m_current;
    Mission.Descent1.StepAlt(i)  = h_mid;
    Mission.Descent1.StepMach(i) = Mach;
    Mission.Descent1.StepCL(i)   = CL_descent_1;
    Mission.Descent1.StepCD(i)   = CD_descent_1;
    Mission.Descent1.StepDrag(i) = D_descent_1;
    Mission.Descent1.StepTreq(i) = T_req_descent_1;
    Mission.Descent1.StepTime(i) = dt;
    Mission.Descent1.StepV(i)    = V_descent_1;
end

Mission.Descent1.Fuel = FuelDescent1;
Mission.Descent1.Time = TimeDescent1;

%% ============================================================
%% 9) Descent 2: 20000 ft to 10000 ft in 500 ft steps
%% ============================================================
h_start = 20000 / SI.ft;
h_end   = 10000 / SI.ft;
dh      = 500 / SI.ft;

nSteps = ceil((h_start - h_end) / dh);

FuelDescent2 = 0;
TimeDescent2 = 0;

Mission.Descent2.StepFuel = zeros(nSteps,1);
Mission.Descent2.StepMass = zeros(nSteps,1);
Mission.Descent2.StepAlt  = zeros(nSteps,1);
Mission.Descent2.StepMach = zeros(nSteps,1);
Mission.Descent2.StepCL   = zeros(nSteps,1);
Mission.Descent2.StepCD   = zeros(nSteps,1);
Mission.Descent2.StepDrag = zeros(nSteps,1);
Mission.Descent2.StepTreq = zeros(nSteps,1);
Mission.Descent2.StepTime = zeros(nSteps,1);
Mission.Descent2.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_high = h_start - (i-1) * dh;
    h_low  = max(h_high - dh, h_end);
    h_mid  = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = V_descent_2 / a;
    W = m_current * g;

    CL_descent_2 = W / (0.5 * rho * V_descent_2^2 * ADP.WingArea);
    CD_descent_2 = ADP.AeroPolar.CD(CL_descent_2);
    D_descent_2  = 0.5 * rho * V_descent_2^2 * ADP.WingArea * CD_descent_2;

    T_idle = IdleFrac * 0.3 * m_current * g;
    T_req_descent_2 = max(T_idle, D_descent_2 - W * ROD_descent_2 / V_descent_2);

    dh_step = h_high - h_low;
    dt = dh_step / ROD_descent_2;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_descent_2 * dt;

    m_current = m_current - FuelStep;
    FuelDescent2 = FuelDescent2 + FuelStep;
    TimeDescent2 = TimeDescent2 + dt;

    Mission.Descent2.StepFuel(i) = FuelStep;
    Mission.Descent2.StepMass(i) = m_current;
    Mission.Descent2.StepAlt(i)  = h_mid;
    Mission.Descent2.StepMach(i) = Mach;
    Mission.Descent2.StepCL(i)   = CL_descent_2;
    Mission.Descent2.StepCD(i)   = CD_descent_2;
    Mission.Descent2.StepDrag(i) = D_descent_2;
    Mission.Descent2.StepTreq(i) = T_req_descent_2;
    Mission.Descent2.StepTime(i) = dt;
    Mission.Descent2.StepV(i)    = V_descent_2;
end

Mission.Descent2.Fuel = FuelDescent2;
Mission.Descent2.Time = TimeDescent2;

%% ============================================================
%% 10) Descent 3: 10000 ft to 1500 ft in 500 ft steps
%% ============================================================
h_start = 10000 / SI.ft;
h_end   = 1500 / SI.ft;
dh      = 500 / SI.ft;

nSteps = ceil((h_start - h_end) / dh);

FuelDescent3 = 0;
TimeDescent3 = 0;

Mission.Descent3.StepFuel = zeros(nSteps,1);
Mission.Descent3.StepMass = zeros(nSteps,1);
Mission.Descent3.StepAlt  = zeros(nSteps,1);
Mission.Descent3.StepMach = zeros(nSteps,1);
Mission.Descent3.StepCL   = zeros(nSteps,1);
Mission.Descent3.StepCD   = zeros(nSteps,1);
Mission.Descent3.StepDrag = zeros(nSteps,1);
Mission.Descent3.StepTreq = zeros(nSteps,1);
Mission.Descent3.StepTime = zeros(nSteps,1);
Mission.Descent3.StepV    = zeros(nSteps,1);

for i = 1:nSteps
    h_high = h_start - (i-1) * dh;
    h_low  = max(h_high - dh, h_end);
    h_mid  = 0.5 * (h_low + h_high);

    [rho, a, ~, ~] = cast.atmos(h_mid);

    Mach = V_descent_3 / a;
    W = m_current * g;

    CL_descent_3 = W / (0.5 * rho * V_descent_3^2 * ADP.WingArea);
    CD_descent_3 = ADP.AeroPolar.CD(CL_descent_3);
    D_descent_3  = 0.5 * rho * V_descent_3^2 * ADP.WingArea * CD_descent_3;

    T_idle = IdleFrac * 0.3 * m_current * g;
    T_req_descent_3 = max(T_idle, D_descent_3 - W * ROD_descent_3 / V_descent_3);

    dh_step = h_high - h_low;
    dt = dh_step / ROD_descent_3;

    TSFC = ADP.Engine.TSFC(Mach, h_mid);
    FuelStep = TSFC * T_req_descent_3 * dt;

    m_current = m_current - FuelStep;
    FuelDescent3 = FuelDescent3 + FuelStep;
    TimeDescent3 = TimeDescent3 + dt;

    Mission.Descent3.StepFuel(i) = FuelStep;
    Mission.Descent3.StepMass(i) = m_current;
    Mission.Descent3.StepAlt(i)  = h_mid;
    Mission.Descent3.StepMach(i) = Mach;
    Mission.Descent3.StepCL(i)   = CL_descent_3;
    Mission.Descent3.StepCD(i)   = CD_descent_3;
    Mission.Descent3.StepDrag(i) = D_descent_3;
    Mission.Descent3.StepTreq(i) = T_req_descent_3;
    Mission.Descent3.StepTime(i) = dt;
    Mission.Descent3.StepV(i)    = V_descent_3;
end

Mission.Descent3.Fuel = FuelDescent3;
Mission.Descent3.Time = TimeDescent3;

FuelDescent = FuelDescent1 + FuelDescent2 + FuelDescent3;
TimeDescent = TimeDescent1 + TimeDescent2 + TimeDescent3;

Mission.Descent.Fuel = FuelDescent;
Mission.Descent.Time = TimeDescent;

%% ============================================================
%% 11) Approach / landing
%% ============================================================
alt_app = 0;
[rho_app, a_app, ~, ~] = cast.atmos(alt_app);

V_app = 145 * 0.514444;
Mach_app = V_app / a_app;
W = m_current * g;

CL_app = W / (0.5 * rho_app * V_app^2 * ADP.WingArea);
CD_app = ADP.AeroPolar.CD(CL_app);
D_app  = 0.5 * rho_app * V_app^2 * ADP.WingArea * CD_app;

T_idle = IdleFrac * 0.3 * m_current * g;
T_req_app = max(T_idle, D_app);

FuelApproach = segmentFuelFromThrust(ADP, Mach_app, alt_app, T_req_app, ApproachTime);
m_current = m_current - FuelApproach;

Mission.ApproachLanding.Fuel = FuelApproach;
Mission.ApproachLanding.Time = ApproachTime;
Mission.ApproachLanding.CL = CL_app;
Mission.ApproachLanding.CD = CD_app;
Mission.ApproachLanding.Drag = D_app;
Mission.ApproachLanding.Treq = T_req_app;
Mission.ApproachLanding.V = V_app;

%% ============================================================
%% 12) Alternate climb
%% ============================================================
FuelAltClimb = segmentFuelFromThrustFrac(ADP, m_current, M_climb, ADP.TLAR.Alt_alternate/2, ClimbFrac, ClimbTime);
m_current = m_current - FuelAltClimb;

Mission.Alternate.Climb.Fuel = FuelAltClimb;
Mission.Alternate.Climb.Time = ClimbTime;

%% ============================================================
%% 13) Alternate cruise
%% ============================================================
[rho_alt, a_alt, ~, ~] = cast.atmos(ADP.TLAR.Alt_alternate);

CL_alt = m_current * g / (0.5 * rho_alt * (a_alt * M_cruise)^2 * ADP.WingArea);
CD_alt = ADP.AeroPolar.CD(CL_alt);
LD_alt = CL_alt / CD_alt;

fs_alt = exp(-AltRange * g * ADP.Engine.TSFC(M_cruise, ADP.TLAR.Alt_alternate) / ...
            (M_cruise * a_alt * LD_alt));

FuelAltCruise = (1 - fs_alt) * m_current;
TimeAltCruise = AltRange / (M_cruise * a_alt);
m_current = m_current * fs_alt;

Mission.Alternate.Cruise.Fuel = FuelAltCruise;
Mission.Alternate.Cruise.Time = TimeAltCruise;

%% ============================================================
%% 14) Alternate descent
%% ============================================================
TimeAltDescent = max(10 * 60, ADP.TLAR.Alt_alternate / 10);
FuelAltDescent = segmentFuelFromThrustFrac(ADP, m_current, M_desc, ADP.TLAR.Alt_alternate/2, DescentFrac, TimeAltDescent);
m_current = m_current - FuelAltDescent;

Mission.Alternate.Descent.Fuel = FuelAltDescent;
Mission.Alternate.Descent.Time = TimeAltDescent;

%% ============================================================
%% 15) Alternate approach
%% ============================================================
FuelAltApproach = segmentFuelFromThrustFrac(ADP, m_current, M_ds, 0, ApproachFrac, ApproachTime);
m_current = m_current - FuelAltApproach;

Mission.Alternate.Approach.Fuel = FuelAltApproach;
Mission.Alternate.Approach.Time = ApproachTime;

%% ============================================================
%% 16) Loiter at 1500 ft for 30 min at minimum drag velocity
%% ============================================================
alt_loiter = 1500 / SI.ft;
[rho_l, a_l, ~, ~] = cast.atmos(alt_loiter);

Cls = 0.2:0.01:2.5;
LDs = Cls ./ ADP.AeroPolar.CD(Cls);
[~, idxL] = max(LDs);

CL_loiter = Cls(idxL);
CD_loiter = ADP.AeroPolar.CD(CL_loiter);
LD_loiter = CL_loiter / CD_loiter;

V_loiter = sqrt((2 * m_current * g) / (rho_l * ADP.WingArea * CL_loiter));
M_loiter = V_loiter / a_l;

FuelLoiter = (1 - exp(-LoiterTime * g * ADP.Engine.TSFC(M_loiter, alt_loiter) / LD_loiter)) * m_current;
m_current = m_current - FuelLoiter;

Mission.Loiter.Fuel = FuelLoiter;
Mission.Loiter.Time = LoiterTime;
Mission.Loiter.CL = CL_loiter;
Mission.Loiter.CD = CD_loiter;
Mission.Loiter.LD = LD_loiter;

%% ============================================================
%% 17) Contingency
%% ============================================================
TripFuel_preCont = FuelTaxiOut + FuelTakeoff + FuelClimb + FuelCruise + ...
                   FuelDescent + FuelApproach;

Fuel5min = (1 - exp(-ContTime * g * ADP.Engine.TSFC(M_loiter, alt_loiter) / LD_loiter)) * m_current;
FuelCont = max(Fuel5min, 0.03 * TripFuel_preCont);

m_current = m_current - FuelCont;

Mission.Contingency.Fuel = FuelCont;
Mission.Contingency.Time = ContTime;

%% ============================================================
%% 18) Totals
%% ============================================================
TripFuel = FuelTaxiOut + FuelTakeoff + FuelClimb + FuelCruise + ...
           FuelDescent + FuelApproach;

ResFuel = FuelAltClimb + FuelAltCruise + FuelAltDescent + ...
          FuelAltApproach + FuelLoiter + FuelCont;

BlockFuel = TripFuel + ResFuel;

MissionTime = TaxiTime_out + TakeoffTime + TimeClimb + TimeCruise + ...
              TimeDescent + ApproachTime + TaxiTime_in + ...
              ClimbTime + TimeAltCruise + TimeAltDescent + ...
              ApproachTime + LoiterTime + ContTime;

Mission.Total.TripFuel = TripFuel;
Mission.Total.ResFuel = ResFuel;
Mission.Total.BlockFuel = BlockFuel;
Mission.Total.Time = MissionTime;

%% ============================================================
%% 19) Mission sanity-check summary
%% ============================================================
Mission.Summary.Takeoff.Time = TakeoffTime;
Mission.Summary.Takeoff.Fuel = FuelTakeoff;
Mission.Summary.Takeoff.Speed = V_to;
Mission.Summary.Takeoff.CL = CL_to;
Mission.Summary.Takeoff.CD = CD_to;
Mission.Summary.Takeoff.Drag = D_to;
Mission.Summary.Takeoff.ThrustRequired = T_req_to;

Mission.Summary.Climb1.Time = TimeClimb1;
Mission.Summary.Climb1.Fuel = FuelClimb1;
Mission.Summary.Climb1.Speed = V_climb_1;
if ~isempty(Mission.TakeoffClimb.StepMach)
    Mission.Summary.Climb1.MachStart = Mission.TakeoffClimb.StepMach(1);
    Mission.Summary.Climb1.MachEnd = Mission.TakeoffClimb.StepMach(end);
    Mission.Summary.Climb1.MeanDrag = mean(Mission.TakeoffClimb.StepDrag);
    Mission.Summary.Climb1.MeanThrustRequired = mean(Mission.TakeoffClimb.StepTreq);
else
    Mission.Summary.Climb1.MachStart = NaN;
    Mission.Summary.Climb1.MachEnd = NaN;
    Mission.Summary.Climb1.MeanDrag = NaN;
    Mission.Summary.Climb1.MeanThrustRequired = NaN;
end

Mission.Summary.Climb2.Time = TimeClimb2;
Mission.Summary.Climb2.Fuel = FuelClimb2;
Mission.Summary.Climb2.Speed = V_climb_2;
if ~isempty(Mission.Climb2.StepMach)
    Mission.Summary.Climb2.MachStart = Mission.Climb2.StepMach(1);
    Mission.Summary.Climb2.MachEnd = Mission.Climb2.StepMach(end);
    Mission.Summary.Climb2.MeanDrag = mean(Mission.Climb2.StepDrag);
    Mission.Summary.Climb2.MeanThrustRequired = mean(Mission.Climb2.StepTreq);
else
    Mission.Summary.Climb2.MachStart = NaN;
    Mission.Summary.Climb2.MachEnd = NaN;
    Mission.Summary.Climb2.MeanDrag = NaN;
    Mission.Summary.Climb2.MeanThrustRequired = NaN;
end

if ~isempty(Mission.Climb3.StepMach)
    Mission.Summary.Climb3.Time = TimeClimb3;
    Mission.Summary.Climb3.Fuel = FuelClimb3;
    Mission.Summary.Climb3.Mach = mean(Mission.Climb3.StepMach);
    Mission.Summary.Climb3.SpeedStart = Mission.Climb3.StepV(1);
    Mission.Summary.Climb3.SpeedEnd = Mission.Climb3.StepV(end);
    Mission.Summary.Climb3.MeanDrag = mean(Mission.Climb3.StepDrag);
    Mission.Summary.Climb3.MeanThrustRequired = mean(Mission.Climb3.StepTreq);
else
    Mission.Summary.Climb3.Time = 0;
    Mission.Summary.Climb3.Fuel = 0;
    Mission.Summary.Climb3.Mach = NaN;
    Mission.Summary.Climb3.SpeedStart = NaN;
    Mission.Summary.Climb3.SpeedEnd = NaN;
    Mission.Summary.Climb3.MeanDrag = NaN;
    Mission.Summary.Climb3.MeanThrustRequired = NaN;
end

Mission.Summary.Cruise.Time = TimeCruise;
Mission.Summary.Cruise.Fuel = FuelCruise;
Mission.Summary.Cruise.Range = RangeCruiseActual;
Mission.Summary.Cruise.Mach = M_cruise;
if any(Mission.Cruise.StepAlt > 0)
    Mission.Summary.Cruise.MeanDrag = mean(Mission.Cruise.StepDrag(Mission.Cruise.StepAlt > 0));
    Mission.Summary.Cruise.MeanThrustRequired = mean(Mission.Cruise.StepTreq(Mission.Cruise.StepAlt > 0));
    idxFirstAlt = find(Mission.Cruise.StepAlt > 0, 1, 'first');
    idxLastAlt  = find(Mission.Cruise.StepAlt > 0, 1, 'last');
    Mission.Summary.Cruise.AltitudeStart = Mission.Cruise.StepAlt(idxFirstAlt);
    Mission.Summary.Cruise.AltitudeEnd = Mission.Cruise.StepAlt(idxLastAlt);
else
    Mission.Summary.Cruise.MeanDrag = NaN;
    Mission.Summary.Cruise.MeanThrustRequired = NaN;
    Mission.Summary.Cruise.AltitudeStart = NaN;
    Mission.Summary.Cruise.AltitudeEnd = NaN;
end

Mission.Summary.Descent1.Time = TimeDescent1;
Mission.Summary.Descent1.Fuel = FuelDescent1;
if ~isempty(Mission.Descent1.StepDrag)
    Mission.Summary.Descent1.MeanDrag = mean(Mission.Descent1.StepDrag);
    Mission.Summary.Descent1.MeanThrustRequired = mean(Mission.Descent1.StepTreq);
else
    Mission.Summary.Descent1.MeanDrag = NaN;
    Mission.Summary.Descent1.MeanThrustRequired = NaN;
end

Mission.Summary.Descent2.Time = TimeDescent2;
Mission.Summary.Descent2.Fuel = FuelDescent2;
if ~isempty(Mission.Descent2.StepDrag)
    Mission.Summary.Descent2.MeanDrag = mean(Mission.Descent2.StepDrag);
    Mission.Summary.Descent2.MeanThrustRequired = mean(Mission.Descent2.StepTreq);
else
    Mission.Summary.Descent2.MeanDrag = NaN;
    Mission.Summary.Descent2.MeanThrustRequired = NaN;
end

Mission.Summary.Descent3.Time = TimeDescent3;
Mission.Summary.Descent3.Fuel = FuelDescent3;
if ~isempty(Mission.Descent3.StepDrag)
    Mission.Summary.Descent3.MeanDrag = mean(Mission.Descent3.StepDrag);
    Mission.Summary.Descent3.MeanThrustRequired = mean(Mission.Descent3.StepTreq);
else
    Mission.Summary.Descent3.MeanDrag = NaN;
    Mission.Summary.Descent3.MeanThrustRequired = NaN;
end

Mission.Summary.Approach.Time = ApproachTime;
Mission.Summary.Approach.Fuel = FuelApproach;
Mission.Summary.Approach.Speed = V_app;
Mission.Summary.Approach.CL = CL_app;
Mission.Summary.Approach.CD = CD_app;
Mission.Summary.Approach.Drag = D_app;
Mission.Summary.Approach.ThrustRequired = T_req_app;

Mission.Summary.Total.MainMissionFuel = TripFuel;
Mission.Summary.Total.ReserveFuel = ResFuel;
Mission.Summary.Total.BlockFuel = BlockFuel;
Mission.Summary.Total.MainMissionTime = TaxiTime_out + TakeoffTime + ...
    TimeClimb + TimeCruise + TimeDescent + ApproachTime + TaxiTime_in;
Mission.Summary.Total.TotalMissionTime = MissionTime;

%% ============================================================
%% 20) Diagnostic plot
%% ============================================================
figure(11); clf;
tiledlayout(2,1)

nexttile
plot(Mission.Cruise.StepCL, Mission.Cruise.StepLD, 'LineWidth', 1.5)
hold on
plot(Mission.Cruise.CL, Mission.Cruise.LD, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5)
xlabel('Lift Coefficient, C_L')
ylabel('Lift-to-Drag Ratio, L/D')
title('Stepped Cruise-Climb')
grid on

nexttile
plot(Cls, LDs, 'LineWidth', 1.5)
xlabel('Lift Coefficient, C_L')
ylabel('Lift-to-Drag Ratio, L/D')
title('Loiter Efficiency Sweep')
grid on

end

function Fuel = segmentFuelFromThrust(ADP, Mach, alt_m, T_req, time_s)
TSFC = ADP.Engine.TSFC(Mach, alt_m);
Fuel = TSFC * T_req * time_s;
end

function Fuel = segmentFuelFromThrustFrac(ADP, mass_kg, Mach, alt_m, thrustFrac, time_s)
g = 9.81;

if ismethod(ADP.Engine, 'Thrust')
    try
        T_avail = ADP.Engine.Thrust(Mach, alt_m);
    catch
        T_avail = 0.3 * mass_kg * g;
    end
else
    T_avail = 0.3 * mass_kg * g;
end

TSFC = ADP.Engine.TSFC(Mach, alt_m);
Fuel = TSFC * thrustFrac * T_avail * time_s;
end