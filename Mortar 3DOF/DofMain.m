% Austen LeBeau
% Aero 3220 - 002

%% Setting up simulation controls

clear;
clc;
t0 = 0;            % initial time
dt = 0.1;          % integration interval in sec
zImpact = 225;     % Impact altitude


%% Initial Conditions

p0 = [ 0; 0; 210];                      % launch point position in ENU frame in meters
az = 75;                                % launch azimuth in deg
el = 50;                                % launch elevation in deg
muzzleVel = 294;                     % mach number of aircraft at launch;
[rho, acousticSpeed] = Density(p0);

%initial velocity vector
v0 = muzzleVel * [cosd(el) * cosd(az); cosd(el) * sind(az); sind(el)];

xVector = [p0 ; v0];                   % initial state vector (position & velocity) 6 states
xDot = dxdt(t0, xVector);              % initial xDot = [velocity and acceleration]. dxdt is a function .

% speed is the magnitude of velocity. Initial speed = speed(1)
speed(1) = norm(xVector(4:6));

% Start recording data for output & plots
output_vector(1, :)= [t0, xVector', xDot(4:6)', 0];

%% Trajectory Computation Loop

t = t0; % start at t = t0.
index = 1;

while true

    [time, x]= ode45(@dxdt,[t t+dt], xVector);
    t = time(end);
    xVector = x(end, :);
    [xDot, accel] = dxdt(t, xVector');                      % finding velocity and acceleration using dxdt
    output_vector(index,:)= [t, xVector, xDot(4:6)', accel];   % xDot(4:6) is acceleration

    speed(index) = norm(xVector(4:6));
    [rho, acousticSpeed] = Density(xVector);

    if xVector(3) < zImpact && xVector(6) < 0

        timeCurve = [output_vector(end - 4, 1), output_vector(end - 3, 1),...
        output_vector(end - 2, 1), output_vector(end - 1, 1), output_vector(end, 1)];
        impactCurve = [output_vector(end - 4, 4), output_vector(end - 3, 4),...
        output_vector(end - 2, 4), output_vector(end - 1, 4), output_vector(end, 4)];

        % Impact functions
        tImpact = interp1(impactCurve, timeCurve, zImpact,'linear');
        deltaT_impact =  tImpact - output_vector(end-1,1);
        yImpact = output_vector(end - 1, 3) + output_vector(end - 1, 6) * deltaT_impact;
        xImpact = output_vector(end - 1, 2) + output_vector(end - 1, 5) * deltaT_impact;
        impPoint = [xImpact yImpact zImpact];
        impactData = [tImpact, impPoint];

        fprintf('%s: %.2f\n', 'Impact Time', tImpact);
        fprintf('%s: %.2f\n', 'Apogee', max(output_vector(:, 4)));
        fprintf('%s: %.2f\n', 'x', xImpact);
        fprintf('%s: %.2f\n', 'y', yImpact);
        fprintf('%s: %.2f\n', 'z', zImpact);

        break;

    end

    index = index + 1;

end


%% Plots and Tables

% Creating new vectors to use with plots; easier to keep track.
tVec = output_vector(:, 1);
x = output_vector(:, 2);
y = output_vector(:, 3);
z = output_vector(:, 4);
A = output_vector(:, 11);


plot_stuff(x, z, 'x', 'z', 'Z vs. X', 1)
plot_stuff(y, z, 'x', 'z', 'Z vs. Y', 2)
plot_stuff(tVec, z, 'Time', 'Z', 'Z vs. Time', 3)
plot_stuff(tVec, speed, 'Time', 'Speed', 'Time vs. Speed', 4)
plot_stuff(tVec, A, 'Time', 'Acceleration', 'Time vs. Acceleration', 5)

% 3D Plot
figure(6)
grid on;
plot3(x, y, z, 'linewidth', 2)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Trajectory')


csvwrite('trajectory.txt',output_vector);


% Function that does plotting so I don't have to keep copying the whole thing.
function plot_stuff(xAx, yAx, xLab, yLab, titleName, fig)
  figure(fig)
  grid on;
  plot(xAx, yAx, 'linewidth', 2)
  xlabel(xLab)
  ylabel(yLab)
  title(titleName)
end
