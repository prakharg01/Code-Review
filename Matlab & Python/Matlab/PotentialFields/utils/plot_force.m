function [] = plot_force(A,clr)
%PLOT_FORCE Summary of this function goes here
%   Detailed explanation goes here
Q=A;
QX = Q(:,1);                                            % Isolate Line ‘x’ Coordinates
QY = Q(:,2);                                            % Isolate Line ‘y’ Coordinates
[~,UV] = gradient(Q);                                   % Generate Gradient
UVX = [UV(1,1); 0];                                     % Define Quiver Arrow ‘x’
UVY = [UV(1,2); 0];                                     % Define Quiver Arrow ‘y’
figure(1)
% plot(QX, QY, '-r', 'LineWidth',1.2)                     % Plot Line (Optional)
%hold on
quiver(QX, QY, UVX, UVY, 0,clr)
%hold off
%axis([-0.15  1.1    -0.15  0.2])
end

