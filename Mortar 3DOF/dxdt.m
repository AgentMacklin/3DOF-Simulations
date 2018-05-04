% Austen LeBeau
% Aero 3220 - 002
% dxdt function

function [xDot, accel] = dxdt(t, xVector)

% Constants
g = 9.8066;
gravity = [0; 0; -g]; % gravity vector
diameter = 0.081;  % m
mass = 4.095;    % kg
area = pi * (diameter / 2)^2;


% Speeds
[density, acousticSpeed] = Density(xVector);
speed = norm(xVector(4:6));
mach = speed / acousticSpeed;


% Data Tables
machTable = [0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9];
dragTable = [.1273 .1267 .1266 .1269 .1276 .1291 .1316 .1359 .1428 .1531];
Cd = interp1(machTable, dragTable, mach, 'linear', 'extrap');


% Accelerations
% drag acceleration in m/sec^2
dragAccel = -0.5 * density * speed * Cd * area * xVector(4:6) / mass;
acceleration = dragAccel + gravity;
xDot = [xVector(4:6); acceleration];
accel = norm(dragAccel);

end
