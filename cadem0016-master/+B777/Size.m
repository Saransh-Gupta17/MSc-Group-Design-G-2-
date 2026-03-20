function [ADP,out] = Size(ADP)

delta = inf;

B777.ConstraintAnalysis(ADP);

while delta > 1
    % Constraint analysis

    % Build geometry
    [~, B7Mass] = B777.BuildGeometry(ADP);

    % Update aero
    B777.UpdateAero(ADP);

    % Mission analysis
    [BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, Mission, CriticalTW, CriticalWS] = ...
    B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

    %Update Thrust/Weight Ratio and required wing loading
    %ADP.WingLoading = CriticalWS.Value;
    ADP.ThrustToWeightRatio = CriticalTW.Value;

    %Update Constraint Analysis
    B777.ConstraintAnalysis(ADP);


    % OEM
    idx = contains([B7Mass.Name],"Fuel","IgnoreCase",true) | ...
          contains([B7Mass.Name],"Payload","IgnoreCase",true);
    ADP.OEM = sum([B7Mass(~idx).m]);

    % Estimate MTOM
    mtom = sum([B7Mass(1:end-2).m]) + ADP.TLAR.Payload + BlockFuel;

    delta = abs(ADP.MTOM - mtom);
    ADP.MTOM = mtom;
end

% Store useful outputs
out.BlockFuel   = BlockFuel;
out.TripFuel    = TripFuel;
out.ResFuel     = ResFuel;
out.Mf_TOC      = Mf_TOC;
out.MissionTime = MissionTime;
out.Mission     = Mission;
out.CriticalTW  = CriticalTW;
out.CriticalWS  = CriticalWS;

% Display once after convergence
fprintf('\n===== Cruise Sizing Study =====\n');
fprintf('Begin cruise TW          = %.4f\n', Mission.CruiseStudy.BeginCruise.TW);
fprintf('Begin cruise WS          = %.1f N/m^2\n', Mission.CruiseStudy.BeginCruise.WS);
fprintf('Begin cruise L/D         = %.2f\n', Mission.CruiseStudy.BeginCruise.LD);

fprintf('\nCruise-climb requirement = 300 ft/min at cruise Mach\n');
fprintf('Cruise-climb TW          = %.4f\n', Mission.CruiseStudy.CruiseClimbRequirement.TW);
fprintf('Cruise-climb WS          = %.1f N/m^2\n', Mission.CruiseStudy.CruiseClimbRequirement.WS);
fprintf('Cruise-climb altitude    = %.1f m\n', Mission.CruiseStudy.CruiseClimbRequirement.Altitude);
fprintf('Cruise-climb Mach        = %.3f\n', Mission.CruiseStudy.CruiseClimbRequirement.Mach);

fprintf('\n===== Critical Sizing Cases =====\n');
fprintf('Critical T/W     = %.4f\n', CriticalTW.Value);
fprintf('Critical T/W seg = %s\n', CriticalTW.Segment);
fprintf('Critical T/W alt = %.1f m\n', CriticalTW.Altitude_m);
fprintf('Critical T/W M   = %.3f\n', CriticalTW.Mach);

fprintf('\nCritical W/S     = %.1f N/m^2\n', CriticalWS.Value);
fprintf('Critical W/S seg = %s\n', CriticalWS.Segment);
fprintf('Critical W/S alt = %.1f m\n', CriticalWS.Altitude_m);
fprintf('Critical W/S M   = %.3f\n', CriticalWS.Mach);

end