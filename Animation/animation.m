%This prorgam receives dimensions from the arudino and simulates it's
%position by changing the simulated robot based on the x and y values it
%recieves from the arduinoIDE. The results showed a triangle moving as a
%response to the input on the wheels of the car. It would rotate if only
%one wheel was turned, and it would move forwards and backwards if both
%wheel

% sets the serial port for communication with arduino
s = serialport("COM3", 9600);
configureTerminator(s, "LF");
flush(s);

%creates all the dimentions
r_width = .5;
r_length = 1;
V = [-r_length/2 -r_length/2 0 r_length/2 0;
     -r_width/2   r_width/2  r_width/2  0  -r_width/2];
%sets up the shape once before the loop
figure
h = fill(nan(1,5), nan(1,5), 'y'); 
axis([-10 10 -10 10])
axis equal
grid on

while true
    %makes it so it only runs when the first line is received
    if s.NumBytesAvailable > 0 || true 
        line = readline(s);
        values = str2double(split(line, ","));

        if numel(values) ~= 4 || any(isnan(values))
            continue   % skip partial lines
        end

        t   = values(1);
        x   = values(2);
        y   = values(3);
        phi = values(4);
        
        %dimentional values
        T = [cos(phi) -sin(phi); sin(phi) cos(phi)];
        pos = [x; y];
        v_c = T*V + pos*ones(1,5);
        
        %draws the chart simulation image
        set(h, 'XData', v_c(1,:), 'YData', v_c(2,:));
        drawnow
    end
end