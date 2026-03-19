function [GeomObj,massObj] = engine(obj)
% engine - this function build the engines for a B777 like aircraft
% engine is based upon a rubberised version of the GE90

obj.Engine = cast.eng.TurboFan.GE90(1,obj.TLAR.Alt_cruise,obj.TLAR.M_c);
obj.Engine = obj.Engine.Rubberise(obj.Thrust/2);

% --------------------------- Create Geometry ----------------------------
Xs = [-0.5,0.5;0.5,0.5;0.5,-0.5;-0.5,-0.5];
Xs = Xs.*[obj.Engine.Length,obj.Engine.Diameter];

% Place engine relative to the aircraft wing position
% Assumes obj.WingPos is the wing quarter-chord/root reference in metres
x_eng = obj.WingPos + 0.25*obj.c_ac;
y_eng = obj.CabinRadius + 1.75*obj.Engine.Diameter;

offsetEng = [x_eng, y_eng];

GeomObj = cast.GeomObj(Name="EngineRight",Xs=Xs+offsetEng);
GeomObj(2) = cast.GeomObj(Name="EngineLeft",Xs=Xs+offsetEng.*[1 -1]);

% ------------------------- Create Mass Objects --------------------------

% engine installation mass (Raymer 15.52)
m_engi = 1.1*(2.575*(obj.Engine.Mass*SI.lb)^0.922)./SI.lb - obj.Engine.Mass;
offsetPylon = offsetEng + [obj.Engine.Length/2,0];

massObj         = cast.MassObj(Name="Engine Right",m=obj.Engine.Mass,X=offsetEng);
massObj(end+1)  = cast.MassObj(Name="Engine Pylon Right",m=m_engi,X=offsetPylon);
massObj(end+1)  = cast.MassObj(Name="Engine Left",m=obj.Engine.Mass,X=offsetEng.*[1 -1]);
massObj(end+1)  = cast.MassObj(Name="Engine Pylon Left",m=m_engi,X=offsetPylon.*[1 -1]);
end