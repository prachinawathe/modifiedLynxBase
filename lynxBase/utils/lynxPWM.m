function q = lynxPWM(lynx, q)

persistent q_prev

if isempty(q_prev)
    q_prev = [0,0,0,0,0,0];
    return;
end

if isempty(lynx.robot)
    disp('Error: must start PWM checks with hardware set to Sim');
    return; 
end

deg_per_us = [.102, .105, .109, .100, .105, .104];
% *8 comes from deadspace
min_diff_rad = (deg_per_us * 2 * pi / 360) * 8;

q_diff = abs(q-q_prev); 

q(q_diff < min_diff_rad) = q_prev(q_diff < min_diff_rad); 

q_prev = q;

end 

