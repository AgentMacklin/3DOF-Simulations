% Austen LeBeau
% Aero 3220 - 002 
% Function that returns atmospheric data.

function [rho, acousticSpeed] = Density(x)

% Constants
    z0 = 0;                 % ground altitude (MSL) where P0 and T0 are measured
    T0 = 288.15;            % degrees K
    P0 = 101325;            % Pascals
    g = 9.8066;             % acceleration due to gravity m/sec^2
    R = 287.058;            % specific gas constant for air J/kg-degree K
    lapseRate = -0.0065;    % lapse rate degree-K/m (z<11 km)
    gammaAir = 1.4;         % adiabatic constant

% Atmosphere Model
    z = x(3);
    T = T0 + lapseRate.*(z-z0);                 % Temperature at altitude z, degree K
    P = P0 .* (T./T0).^(-g./(lapseRate.*R)) ;   % Pressure at altitude z, Pascals
    rho = P ./ (R.*T);                          % Density at altitude z, kg/m^3
    acousticSpeed = sqrt(gammaAir .* R .* T);   % Acoustic speed, m/sec
    
end