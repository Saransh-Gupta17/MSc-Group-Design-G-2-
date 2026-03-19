function [ThrustToWeightRatio,WingLoading] = ConstraintAnalysis(obj)

%% estimate T/W and W/S from constraint analysis

% set Wing Area and Thrust
SweepQtrChord = real(acosd(0.75.*obj.Mstar./obj.TLAR.M_c)); % quarter chord sweep angle
obj.WingArea = obj.MTOM*9.81 / obj.WingLoading / cosd(SweepQtrChord);
obj.Thrust = obj.ThrustToWeightRatio * obj.MTOM * 9.81;

% constant aspect ratio -> span scales with wing area
obj.Span = sqrt(obj.AR * obj.WingArea);

ThrustToWeightRatio = obj.ThrustToWeightRatio;
WingLoading = obj.WingLoading;

end