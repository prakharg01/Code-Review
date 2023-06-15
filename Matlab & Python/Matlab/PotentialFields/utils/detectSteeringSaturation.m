function [state,isterminal,direction] = detectSteeringSaturation(t,y)
  maxSteerAngle = pi/4;               % Maximum steering angle (pi/4 radians)
  state(4)= y(4)^2-pi/16;
  %state(4) = (y(4) - maxSteerAngle);   % Saturation event occurs when the 4th state, theta, is equal to the max steering angle    
  isterminal(4) = 1;                   % Integration is terminated when event occurs 
  direction(4) = 0;                    % Bidirectional termination 
end

