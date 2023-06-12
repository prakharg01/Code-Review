% [X,Y] = meshgrid(linspace(-5, 5, 50));              % Create Mesh Data
% fcn = @(x,y,k) 0.5*((x-k).^2 + (y-k).^2).^0.5;                       % Function To Plot (Vary Sign Of ‘x’)
% v = [1:-0.05:-1;  -1:0.05:1];                       % Multiplier For ‘x’
% for k1 = 1:2                                        % One Cycle For ‘Up’, One FOr ‘Down’
%     for k2 = v(k1,:)                                % Select A Direction From ‘v’
%         surfc(X, Y, fcn(X,Y,k2))                    % Draw Surface
%         axis([-5  5    -5  5    -30  50])           % Set ‘axis’ For All Plots
%         drawnow                                     % Draw Plot
%         pause(0.1)                                  % Create Evenly-Timed Steps For Animation
%     end
% end
k=-4;
while(k~=2)
[X,Y] = meshgrid(linspace(-10, 10, 50));% Create Mesh Data
 fcn =@(x,y,k) 2*((x).^2+y.^2)+2*((x).^2+y.^2);
%fcn = @(x,y,k) 82*exp(-(x-k).^2/2-(y-k).^2/2)+2.5*2*(((x-1).^2)+(y.^2)).^0.5 ; % Function To Plot (Vary Sign Of ‘x’)
%fcn = @(x,y,k) 42*exp(-(x-k).^2/2-(y-k).^2/2)+22*exp(-(x-k-1.5).^2/2-(y-k-1.5).^2/2)+22*exp(-(x-k-3).^2/2-(y-k-3).^2/2)+22*exp(-(x-k-4.5).^2/2-(y-k-4.5).^2/2);% Function To Plot (Vary Sign Of ‘x’)
%fcn = @(x,y,k) 0.5*1./(((x-k).^2 + (y-1).^2).^2.5)+2.5*1./(((x-1).^2 + (y-k).^2).^2.5)+0.5*((x-k).^2 + (y).^2).^1+15*exp(-0.8*(sqrt((y+k).^2+((x+k).^2))-1.4));                     % Function To Plot (Vary Sign Of ‘x’)
%v = [1:-0.05:-1;  -1:0.05:1];                       % Multiplier For ‘x’
                                                    % One Cycle For ‘Up’, One FOr ‘Down’
    %for k2 = -3:3                               % Select A Direction From ‘v’
        surfc(X, Y, fcn(X,Y,k));            % Draw Surface
        axis([-10 10   -10  10  -50  50])           % Set ‘axis’ For All Plots
        drawnow                                     % Draw Plot
        pause(0.1)    % Create Evenly-Timed Steps For Animation
    %end
    k=k+0.5;
end
% syms x y ;
% k=0.5;
% while(k~=4)
% [X, Y] = meshgrid(eps:0.1:5,eps:0.1:5);
% f=2.5*1./(((x-k-0.01).^2 + (y-1-0.01).^2).^2.5)+2.5*1./(((x-1-0.01).^2 + (y-k-0.01).^2).^2.5)+0.5*((x-k).^2 + (y).^2).^1;
% g=gradient(f,[x;y]);
% g1= subs(g(1), [x y], {X,Y});
% g2= subs(g(2), [x y], {X,Y});
% quiver(X,Y,g1,g2);
% drawnow
% pause(0.01)
% k=k+0.5;
% end
