%% Bode Lamb
% This is a MATLAB script that plots our simulation Data from Simulink and
% can plot it against obtained data. This simulink data was used in order
% to obtain the correct PI values, and was tested against our obtained data
% to ensure accuracy. No hardware connections are required
%

hold on
grid on
xlabel('Time (s)')
ylabel('Position')
title('Position Response')
plot(out.position)

