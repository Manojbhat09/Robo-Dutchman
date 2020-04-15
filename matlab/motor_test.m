%% Add Hebi to library path
currentDir = fileparts(mfilename('fullpath'));
addpath(fullfile(currentDir , 'lib', 'hebi'));

%% Initialize robot and cmd
robot = HebiLookup.newGroupFromNames('RoboDutchman', {'LeftWheel','RightWheel'});
cmd = CommandStruct();

%% Initialize attributes
wheel_radius = 0.0635; % 2.5in = 0.0635m
wheel_separation = 0.2286; % 9in = 0.2286m
state = [0 0 0]; % [x, y, th]
prev_pos = 0;

%% Main Dead Reckoning Loop
while (true)   
    fbk = robot.getNextFeedback();
    
    % verify feedback working as expected
    pos = fbk.position;
    pos(2) = pos(2) * (-1);
    disp(pos);
    
    % disregard initial update
    if prev_pos == 0
        prev_pos = pos;
        continue;
    end
    
    % find diffs
    left_diff = pos(1) - prev_pos(1);
    left_diff = left_diff * wheel_radius;
    
    right_diff = pos(2) - prev_pos(2);
    right_diff = right_diff * wheel_radius;
    
    prev_pos = pos;
    
    % determine angular and linear velocities
    v = (right_diff + left_diff)/2;
    w = (right_diff - left_diff)/wheel_separation;
    
    % runge-kutta state update
    % (https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html)
    th = state(3);
    
    k00 = v*cos(th);
    k01 = v*sin(th);
    k02 = w;
    
    k10 = v*cos(th + (1/2)*k02);
    k11 = v*sin(th + (1/2)*k02);
    k12 = w;
    
    k20 = v*cos(th + (1/2)*k12);
    k21 = v*sin(th + (1/2)*k12);
    k22 = w;
    
    k30 = v*cos(th + k22);
    k31 = v*sin(th + k22);
    k32 = w;
    
    state(1) = state(1) + (1/6)*(k00 + 2*(k10 + k20) + k30);
    state(2) = state(2) + (1/6)*(k01 + 2*(k11 + k21) + k31);
    state(3) = state(3) + (1/6)*(k02 + 2*(k12 + k22) + k32);
    
    disp(state);
end