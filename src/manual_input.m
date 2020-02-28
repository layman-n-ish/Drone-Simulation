function input = manual_input(dt, T)
    % Compute control inputs (angular velocities of the motors) at an instant
    
    hovering_speed = 385157.718; % (rad/s)^2

    % Hover at a height for first T seconds
    t = 0:dt:T-dt; 
    w1_1 = hovering_speed + 30000 * sin((2*pi*t)/T);
    w2_1 = hovering_speed + 30000 * sin((2*pi*t)/T);
    w3_1 = hovering_speed + 30000 * sin((2*pi*t)/T);
    w4_1 = hovering_speed + 30000 * sin((2*pi*t)/T);
    
    wp1 = [w1_1; w2_1; w3_1; w4_1];
    
    % Roll for next T seconds
    t = T:dt:2*T-dt;
    w1_2 = repmat(hovering_speed, 1, size(t, 2)); % hovering speed
    w2_2 = hovering_speed + 3000 * sin((2*pi*t)/T);
    w3_2 = repmat(hovering_speed, 1, size(t, 2));
    w4_2 = hovering_speed + -3000 * sin((2*pi*t)/T);
    
    wp2 = [w1_2; w2_2; w3_2; w4_2];
    
    % Pitch for next T seconds
    t = 2*T:dt:3*T-dt;
    w1_3 = hovering_speed + 3000 * sin((2*pi*t)/T);
    w2_3 = repmat(hovering_speed, 1, size(t, 2));
    w3_3 = hovering_speed + -3000 * sin((2*pi*t)/T);
    w4_3 = repmat(hovering_speed, 1, size(t, 2));
    
    wp3 = [w1_3; w2_3; w3_3; w4_3];
    
    % Yaw for next T seconds
    t = 3*T:dt:4*T-dt;
    w1_4 = hovering_speed + 10000 * sin((2*pi*t)/T);
    w2_4 = hovering_speed + -10000 * sin((2*pi*t)/T); 
    w3_4 = hovering_speed + 10000 * sin((2*pi*t)/T);
    w4_4 = hovering_speed + -10000 * sin((2*pi*t)/T);
    
    wp4 = [w1_4; w2_4; w3_4; w4_4];
    
    input = [wp1, wp2, wp3, wp4];
    
end