classdef AeroPolar
    %AEROPOLAR Class to estimate the drag coefficient of a B777-like
    %aircraft during flight

    properties
        Beta
        e
        CD0
        CDmin
        CLmin   % CL at minimum drag
        AR = 9  % Initial guess for aspect ratio
    end

    methods
        function obj = AeroPolar(ADP)
            % CD0 estimate
            obj.CD0 = 0.019;



            % Calc induced factor
            % Reference: 10.2514/1.C036529 Eq.4
            Q = 1.05;
            P = 0.007;

            obj.e = 1 / (Q + P*pi*obj.AR);   % Oswald efficiency factor
            obj.Beta = 1 / (pi*obj.AR*obj.e);
        end

        function CD = CD(obj,CL)
            % Calc CD for a given CL
            CD = obj.CD0 + obj.Beta*CL.^2;
        end
    end
end