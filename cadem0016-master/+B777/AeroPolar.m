classdef AeroPolar
    %AEROPOLAR Drag polar for a B777-like aircraft
    %
    % Supports two usage modes:
    %   1) AeroPolar(ADP)
    %      -> simple baseline polar for legacy calls / setup
    %
    %   2) AeroPolar(ADP, rho, mu, V)
    %      -> higher-fidelity parasite drag build-up at a specific
    %         flight condition, plus induced drag

    properties
        Beta
        e
        CD0
        CDmin
        CLmin

        % Optional drag build-up breakdown
        CD_wing = NaN
        CD_fuse = NaN
        CD_misc = NaN

        % Reference / geometry
        Sref = NaN
        Swet_wing = NaN
        Swet_fuse = NaN
        AR = NaN
        L_fuse = NaN
        D_fuse = NaN

        % Flight condition
        Mach = NaN
        rho = NaN
        mu = NaN
        V = NaN

        % Reynolds numbers
        Re_wing = NaN
        Re_fuse = NaN
    end

    methods
        function obj = AeroPolar(ADP, rho, mu, V)

            % ---------------- Basic induced drag model ----------------
            obj.AR = ADP.AR;

            Q = 1.05;
            P = 0.007;

            obj.e = 1 / (Q + P*pi*obj.AR);
            obj.Beta = 1 / (pi * obj.AR * obj.e);

            % ---------------- Legacy / setup mode ----------------
            % Allows B777.UpdateAero(ADP) to keep working
            if nargin < 4
                obj.CD0 = 0.019;
                obj.CDmin = obj.CD0;
                obj.CLmin = 0.0;
                return
            end

            % ---------------- Store flight condition ----------------
            obj.rho = rho;
            obj.mu  = mu;
            obj.V   = V;

            % ---------------- Reference geometry ----------------
            obj.Sref = getADPField(ADP, {'WingArea','S_ref','S'}, NaN);
            obj.L_fuse = getADPField(ADP, {'FuselageLength','L_fuse','Length'}, NaN);
            obj.D_fuse = getADPField(ADP, {'FuselageDiameter','D_fuse','Diameter'}, NaN);

            if isnan(obj.Sref) || obj.Sref <= 0
                error('B777.AeroPolar:MissingWingArea', ...
                    'Could not find a valid wing reference area in ADP.');
            end

            if isnan(obj.L_fuse) || obj.L_fuse <= 0
                % sensible B777-like fallback
                obj.L_fuse = 63.7;
            end

            if isnan(obj.D_fuse) || obj.D_fuse <= 0
                % sensible widebody fallback
                obj.D_fuse = 6.2;
            end

            % ---------------- Estimate Mach ----------------
            % If no sound speed is supplied, use a standard approximation
            a = getADPField(ADP, {'a_sound','SoundSpeed'}, NaN);
            if isnan(a) || a <= 0
                a = 340.0;
            end
            obj.Mach = V / a;

            % ---------------- Wetted areas ----------------
            obj.Swet_wing = 2.05 * obj.Sref;
            obj.Swet_fuse = pi * obj.D_fuse * obj.L_fuse;

            % ---------------- Reference chord ----------------
            c_ref = getADPField(ADP, {'MAC','MeanAerodynamicChord'}, NaN);
            if isnan(c_ref) || c_ref <= 0
                c_ref = sqrt(obj.Sref / obj.AR);
            end

            % ---------------- Reynolds numbers ----------------
            obj.Re_wing = rho * V * c_ref / mu;
            obj.Re_fuse = rho * V * obj.L_fuse / mu;

            % ---------------- Skin-friction coefficients ----------------
            Cf_wing = skinFrictionCoeff(obj.Re_wing, obj.Mach);
            Cf_fuse = skinFrictionCoeff(obj.Re_fuse, obj.Mach);

            % ---------------- Form factors ----------------
            FF_wing = 1.35;
            Q_wing  = 1.0;

            fineness = obj.L_fuse / obj.D_fuse;
            FF_fuse = 1 + 60/(fineness^3) + fineness/400;
            Q_fuse  = 1.0;

            % ---------------- Parasite drag build-up ----------------
            obj.CD_wing = Cf_wing * FF_wing * Q_wing * (obj.Swet_wing / obj.Sref);
            obj.CD_fuse = Cf_fuse * FF_fuse * Q_fuse * (obj.Swet_fuse / obj.Sref);

            % Small lumped allowance for tails / nacelles / excrescences
            obj.CD_misc = 0.0035;

            obj.CD0 = obj.CD_wing + obj.CD_fuse + obj.CD_misc;
            obj.CDmin = obj.CD0;
            obj.CLmin = 0.0;
        end

        function CD = CD(obj, CL)
            CD = obj.CD0 + obj.Beta * CL.^2;
        end
    end
end

%% ---------------- Local helper functions ----------------

function Cf = skinFrictionCoeff(Re, M)
Re = max(Re, 1e5);
Cf = 0.455 / (((log10(Re))^2.58) * (1 + 0.144*M^2)^0.65);
end

function val = getADPField(ADP, names, defaultVal)
val = defaultVal;

for i = 1:numel(names)
    name = names{i};

    try
        if isobject(ADP) || isstruct(ADP)
            if isprop(ADP, name) || isfield(ADP, name)
                tmp = ADP.(name);
                if isnumeric(tmp) && isscalar(tmp) && ~isempty(tmp)
                    val = tmp;
                    return
                end
            end
        end
    catch
    end
end
end