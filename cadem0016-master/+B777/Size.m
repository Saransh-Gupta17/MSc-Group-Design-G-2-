function [ADP,out] = Size(ADP)
% interatively build the model, run mission analysis and estimate required
% MTOM untill covnergence

delta = inf;

while delta > 1
    % constraint Analysis
    B777.ConstraintAnalysis(ADP);
    
    % build geometry
    
    [~,B7Mass] = B777.BuildGeometry(ADP);
    
    % update Aero
    B777.UpdateAero(ADP);
    
    % mission Analysis
    [BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, Mission] = ...
    B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

    disp(Mission.Summary)


    
    % calc OEM
    idx = contains([B7Mass.Name],"Fuel","IgnoreCase",true) | ...
          contains([B7Mass.Name],"Payload","IgnoreCase",true);

    ADP.OEM = sum([B7Mass(~idx).m]);
    
    % estimate MTOM
    idxFuelPayload = contains([B7Mass.Name],"Fuel","IgnoreCase",true) | ...
                 contains([B7Mass.Name],"Payload","IgnoreCase",true);
    mtom = sum([B7Mass(1:end-2).m])+ADP.TLAR.Payload+BlockFuel;
    delta = abs(ADP.MTOM - mtom);
    ADP.MTOM = mtom;

    ADP.Mf_Fuel = BlockFuel / ADP.MTOM;
    ADP.Mf_TOC  = Mf_TOC;
    ADP.Mf_Ldg  = (ADP.MTOM - TripFuel) / ADP.MTOM;
    ADP.Mf_res  = ResFuel / ADP.MTOM;
    
    % estimate output parameters
    out = struct();
    out.BlockFuel = BlockFuel;
    out.DOC = BlockFuel * 1;
    out.ATR = BlockFuel;
    out.MissionTime = MissionTime;
end
fprintf('\n===== Mission Summary =====\n');
fprintf('Takeoff:  Time = %.1f s, Fuel = %.1f kg, Speed = %.1f m/s\n', ...
    Mission.Summary.Takeoff.Time, ...
    Mission.Summary.Takeoff.Fuel, ...
    Mission.Summary.Takeoff.Speed);

fprintf('Climb 1:  Time = %.1f s, Fuel = %.1f kg\n', ...
    Mission.Summary.Climb1.Time, ...
    Mission.Summary.Climb1.Fuel);

fprintf('Climb 2:  Time = %.1f s, Fuel = %.1f kg\n', ...
    Mission.Summary.Climb2.Time, ...
    Mission.Summary.Climb2.Fuel);

fprintf('Climb 3:  Time = %.1f s, Fuel = %.1f kg\n', ...
    Mission.Summary.Climb3.Time, ...
    Mission.Summary.Climb3.Fuel);

fprintf('Cruise:   Time = %.1f s, Fuel = %.1f kg, Range = %.1f km\n', ...
    Mission.Summary.Cruise.Time, ...
    Mission.Summary.Cruise.Fuel, ...
    Mission.Summary.Cruise.Range/1000);

fprintf('Descent1: Time = %.1f s, Fuel = %.1f kg\n', ...
    Mission.Summary.Descent1.Time, ...
    Mission.Summary.Descent1.Fuel);

fprintf('Descent2: Time = %.1f s, Fuel = %.1f kg\n', ...
    Mission.Summary.Descent2.Time, ...
    Mission.Summary.Descent2.Fuel);

fprintf('Descent3: Time = %.1f s, Fuel = %.1f kg\n', ...
    Mission.Summary.Descent3.Time, ...
    Mission.Summary.Descent3.Fuel);

fprintf('Approach: Time = %.1f s, Fuel = %.1f kg, Speed = %.1f m/s\n', ...
    Mission.Summary.Approach.Time, ...
    Mission.Summary.Approach.Fuel, ...
    Mission.Summary.Approach.Speed);

fprintf('\nMain mission fuel = %.1f kg\n', Mission.Summary.Total.MainMissionFuel);
fprintf('Reserve fuel      = %.1f kg\n', Mission.Summary.Total.ReserveFuel);
fprintf('Block fuel        = %.1f kg\n', Mission.Summary.Total.BlockFuel);
fprintf('Main mission time = %.1f min\n', Mission.Summary.Total.MainMissionTime/60);
fprintf('Total mission time= %.1f min\n', Mission.Summary.Total.TotalMissionTime/60);
end