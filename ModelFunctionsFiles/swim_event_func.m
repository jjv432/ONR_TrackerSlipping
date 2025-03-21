% Event function to detect liftoff
function [value,isterminal,direction] = swim_event_func(t,q)

    global Fnormal;      % Stop when Fn = 0
    value = Fnormal;
    isterminal = 1;  % Stop integration when event occurs
    direction = -1;  % Detect when Fn crosses zero from positive to negative
end