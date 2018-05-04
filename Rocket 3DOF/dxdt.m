% Austen LeBeau
% Aero 3220 - 002
% dxdt function

function [xDot, dragForce] = dxdt(t, xVector)

% Constants
gravity = [0; 0; -9.8066];
diameter = 0.070;
area = pi * (diameter / 2)^2;
[density, acousticSpeed] = Density(xVector);
mach = norm(xVector(4:6)) / acousticSpeed;

% Data Tables

thrust = [6.920452 6.920452 6.153180 5.981393 5.858088 5.835056 ...
              6.132371 6.602429 7.072488 7.302045 8.760003 2.963190 ...
              0.814937 0.0 0.0].*10^3;

mass = [0.0104319 0.0101886 0.0098901 0.0096176 0.0093507 ...
            0.0090666 0.0087719 0.0084845 0.0081759 0.0078474 ...
            0.0074813 0.0071776 0.0071545 0.0071368 0.0071368].*10^3;

time = [0.0 0.0000780 0.0001793 0.0002793 0.0003793 0.0004821 ...
            0.0005821 0.0006821 0.0007821 0.0008821 0.0009821 ...
            0.0010821 0.0011120 0.0011130 0.1000000].*10^3;

coast_cd = [0.7000 0.7031 0.7357 0.7711 0.8122 0.8791 0.9615 0.9988 ...
            1.0106 1.0053 0.9983 0.9647 0.9115 0.8543 0.8340 0.8000];

coast_mach = [0.1000 0.7841 0.8258 0.8616 0.9024 0.9500 1.0027 1.0956 ...
              1.2100 1.3371 1.4028 1.6031 1.8107 2.0240 2.1066 2.2500];

boost_cd = [0.5500 0.5502 0.5509 0.5505 0.5684 0.6841 0.7479 0.7555 ...
            0.7367 0.6925 0.6624 0.6611 0.6400];

boost_mach = [0.1000 0.3019 0.4655 0.6295 0.8082 0.9984 1.1866 1.3931 ...
              1.6186 1.8782 2.0964 2.1066 2.2500];

% Masses, Thrusts, and Drags

currentThrust = interp1(time, thrust, t, 'linear', 'extrap');

currentMass = interp1(time, mass, t, 'linear', 'extrap');

if currentThrust == 0

    dragCoeff = interp1(coast_mach, coast_cd, mach, 'linear','extrap'); % Coast

elseif currentThrust > 0

    dragCoeff = interp1(boost_mach, boost_cd, mach, 'linear', 'extrap'); % Boost

end


% Accelerations
dragAccel = -0.5 .* density.* norm(xVector(4:6)) .* dragCoeff .* area .* xVector(4:6) ./ currentMass;
thrustAccel = currentThrust .* xVector(4:6) ./ (norm(xVector(4:6)) .* currentMass);
acceleration = dragAccel + thrustAccel + gravity;
xDot = [xVector(4:6); acceleration];
dragForce = abs(-0.5 .* density .* norm(xVector(4:6)).^2 .* dragCoeff .* area);

end
