% Austen LeBeau
% Aero 3220 - 002

%% Setting up simulation controls

clear;
clc;
t0 = 0;            % initial time  
dt = 0.01;         % integration interval in sec 
zImpact = 0;       % Impact altitude 

%% Initial Conditions 

p0 = [ 0; 0; 1500];                     % launch point position in ENU frame in meters 
launchMach = 0.5 ;                      % mach number of aircraft at launch; 
machNumber(1) = launchMach;             % start saving Mach number for output 
[rho,acousticSpeed] = Density(p0); 
speed0 = acousticSpeed * launchMach;    % aircraft speed at launch 
psi0 = 0;                               % launch azimuth in deg 
theta0 = -30;                           % launch elevation in deg 

v0 = speed0.*[cosd(theta0)*sind(psi0); 
     cosd(theta0)*cosd(psi0); sind(theta0)]; % initial velocity vector 
          
xVector = [ p0 ; v0 ];                 % initial state vector (position & velocity) 6 states 
xDot = dxdt (t0,xVector);              % initial xDot = [velocity and acceleration]. dxdt is a function . 

% speed is the magnitude of velocity. Initial speed = speed(1) 
speed(1) = norm(xVector(4:6));

% Start recording data for output & plots 
output_vector(1, :)= [t0,xVector',xDot(4:6)']; 
disp(output_vector);                       

%% Trajectory Computation Loop 

t = t0; % start at t = t0. 
index = 1;

while true 

    [time,x]= ode45(@dxdt,[t t+dt], xVector);
    t = time(end); 
    xVector = x(end, :);   
    [xDot, df] = dxdt(t,xVector');                      % finding velocity and acceleration using dxdt 
    output_vector(index,:)= [t, xVector, xDot(4:6)'];   % xDot(4:6) is acceleration

    speed(index) = norm(xVector(4:6));
    [rho,acousticSpeed] = Density(xVector); 
    machNumber(index) = speed(index)/acousticSpeed; 
    
    FlightAngle(1) = atand(xVector(6)/sqrt(xVector(4)^2+xVector(5)^2));
    FlightAngle(index) = atand(xVector(6)/sqrt(xVector(4)^2+xVector(5)^2)); 
    dragForce(index) = df;
    
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
        impPoint = [xImpact, yImpact, zImpact]; 
        impactData = [tImpact, impPoint]; 
        break;
        
    end
    
    index = index + 1;

end

%% Plots and Tables

acceleration = sqrt(output_vector(1:end,8).^2 + output_vector(1:end,9).^2 + output_vector(1:end, 10).^2);
csvwrite('Output Vector.csv', output_vector); 

% east vs time
plot(output_vector(1:end,4), output_vector(1:end,2));
title('East Coordinates vs. Altitude');
xlabel('Altitude');
ylabel('East Coordinates');
grid on
figure;

% north vs time
plot(output_vector(1:end,4), output_vector(1:end,3));
title('North Cooridinates vs. Time');
xlabel('Altitude');
ylabel('North Coordinates');
grid on
figure;

% altitude vs time
plot(output_vector(1:end,1), output_vector(1:end,4));
title('Altitude vs time');
xlabel('Time')
ylabel('Altitude')
grid on
figure;

% 3D graph
plot3(output_vector(1:end,2),output_vector(1:end,3),output_vector(1:end,4));
title('ENU Plot');
xlabel('East');
ylabel('North');
zlabel('Altitude');
grid on
figure;

% speed vs time
plot(output_vector(1:end,1), speed);
title('Speed vs. Time');
xlabel('Time');
ylabel('Speed (m/s)');
grid on
figure;

% plot vs time
plot(output_vector(1:end,1),machNumber);
title('Mach Number vs. Time');
xlabel('Time');
ylabel('Speed');
grid on
figure;

plot(output_vector(1:end,1),acceleration);
title('Acceleration Magnitude vs. Time');
xlabel('Time');
ylabel('Acceleration Magnitude');
grid on
figure;

plot(output_vector(1:end,1), dragForce(1:end));
title('drag Force vs time');
xlabel('Time');
ylabel('Drag Force');
grid on
figure;

plot(output_vector(1:end,1), FlightAngle);
title('Flight Path Angle vs. Time');
xlabel('Time');
ylabel('FPA');
grid on








