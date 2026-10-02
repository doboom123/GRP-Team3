%% Bode Lamb
% This is a MATLAB script that plots our simulation Data from Simulink and
% can plot it against obtained data. This simulink data was used in order
% to obtain the correct PI values, and was tested against our obtained data
% to ensure accuracy. No hardware connections are required
% 
% The Simulink file shared in this folder models our motor as a transfer
% function in its own subsystem. Gains exist inside and outside of the loop
% as well if you want to tinker there as well. Outside of the subsystem, we
% have a step input into a gain where the user can set what position they
% want. Then it goes into a PI controller where the user can adjust inputs
% to find desired PI values. There are also scopes at all points so you can
% see all values like speed, position, and voltage.
%

hold on
grid on
xlabel('Time (s)')
ylabel('Position')
title('Position Response')
plot(out.position)

