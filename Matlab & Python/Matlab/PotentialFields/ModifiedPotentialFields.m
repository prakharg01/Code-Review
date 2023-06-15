clear all;
clc;
close all;

% Set figure properties
figure('WindowState', 'maximized');

% Parameters for controller and planners
t_simulator = 0.1;          % Simulator frequency
v_max = 5;                  % Maximum linear velocity
w_max = 5;                  % Maximum angular velocity
m = 2;                      % Attractive force parameter
n = 2;                      % Attractive force parameter
param_local_g = 0.2;        % Hybrid parameter
local_var = 0.4;            % Hybrid parameter
alpha_global = 0.05;        % Global force attractive parameter
beta_global = 0.005;        % Global force attractive parameter
eta1_global = 1;            % Global force repulsive parameter
eta2_global = 1;            % Global force repulsive parameter
eta1_local = 10.0;          % Local force repulsive parameter
eta2_local = 10.0;          % Local force repulsive parameter
kp = 1;                     % Controller parameter
ki = 0;                     % Controller parameter
kd = 0.004;                 % Controller parameter
c = 1.99;                   % Controller parameter
l_car = 1;                  % Length of robotic base
b_car = 0.5;                % Breadth of robotic base
a_max = 1.5;                % Maximum deceleration for safety and comfort
global_goal_radius = 0.5;   % Attractive goal radius
x1_b = 25;                  % Params for tuning ranges and Gaussians
x2_b = 15;                  % Params for tuning ranges and Gaussians
x3_b = 10;                  % Params for tuning ranges and Gaussians
beta_hill = 15;             % Params for tuning ranges and Gaussians

int_error = 0;
d_error = 0;
prev_error = 0;
prev_var = 0;
prev_d_min = 0;

% Car kinematics setup
kinematicModel = ackermannKinematics;
options = odeset('Events', @detectSteeringSaturation);
initialState = [0 0 0 0];
state_car = initialState;
cmds = [0.1 0];
input_car = cmds;
v = 0;

% Obstacles and target setup
% Dynamic obstacles
kinematicModel_obs = bicycleKinematics;
initial_state_obs = [10 0 pi; 10 -13 -pi/4; 19 0 -pi/3]; %[x, y, theta]
state_obs = initial_state_obs;
ox = state_obs(:, 1)';
oy = state_obs(:, 2)';
input_obs = [0 0; 0 0; 0 0];
o_size = [2, 2, 2];         % Obstacle sizes
velocity_obstacle = [2; -1; -2];   % Input velocity
steer_obstacle = [0; 0; 0];        % Input steering angle
v_o = [0, 0; 0, 0; 0, 0];
n_dynamic_obs = numel(o_size);

% Static obstacles
ox_static = [10, 30, 30];
oy_static = [5, -6, 6];
o_size_static = [9, 9, 9, 9, 5] * 0.3;
v_o_s = zeros(2, numel(ox_static));

% Target velocity
v_t = [0; 0];
p_f = [49; 0];
kinematicModel_target = ackermannKinematics;
initialState_target = [p_f(1), p_f(2), 0.01, 0];
state_target = initialState_target;
v_target = 0;
steer_target = 0;
input_target = [v_target, steer_target];

% Start simulation
dt = 0;
c_x = [];
c_y = [];

VRO = [];

while norm(state_car(1:2)' - p_f) > global_goal_radius
    % Global force planner
    v_car = [input_car(1) * cos(state_car(3)); input_car(1) * sin(state_car(3))];
    v_o = [input_obs(:, 1) .* cos(state_obs(:, 3)), input_obs(:, 1) .* sin(state_obs(:, 3))]';
    v_t = [input_target(1) * cos(state_target(3)), input_target(1) * sin(state_target(3))]';
    n_rt = (p_f - state_car(1:2)') / norm(p_f - state_car(1:2)');
    d_g = norm(p_f - state_car(1:2)');
    v_rt = (-v_car + v_t) / norm(-v_car + v_t);
    f_attractive1 = m * alpha_global * (norm(p_f - state_car(1:2)'))^(m-1) * n_rt;
    f_attractive2 = n * beta_global * (norm(v_t - v_car))^(n-1) * v_rt;
    f_attractive = f_attractive1;
    f_rep = [0; 0];
    f_repulsive = [0; 0];
    OX = [ox, ox_static];
    OY = [oy, oy_static];
    O_SIZE = [o_size, o_size_static];
    V_O = [v_o, v_o_s];
    n_obs = 1:(numel(ox) + numel(ox_static));
    d_min = [];
    F_VAR = [];
    
    for j = n_obs
        d = norm([OX(j); OY(j)] - [state_car(1); state_car(2)]);
        d_min = [d_min, d - O_SIZE(j)];
        nro = (-[state_car(1); state_car(2)] + [OX(j); OY(j)]) / d;
        nro_perp_1 = [-nro(2); nro(1)];
        nro_perp_2 = [nro(2); -nro(1)];
        
        % Choose the best option of nro_perp
        if nro_perp_1' * V_O(:, j) < 0
            nro_perp = nro_perp_1;
        end
        
        if nro_perp_2' * V_O(:, j) < 0
            nro_perp = nro_perp_2;
        end
        
        if (nro_perp_1' * V_O(:, j) == 0) && (nro_perp_2' * V_O(:, j)) == 0
            if nro_perp_1' * f_attractive >= 0
                nro_perp = nro_perp_1;
            else
                nro_perp = nro_perp_2;
            end
        end
        
        vro = ((v_car - V_O(:, j))') * nro;
        VRO = [VRO, vro];
        vro_perp = sqrt((norm(v_car - V_O(:, j))^2) - vro^2);
        ro_vro = (vro * vro) / (2 * a_max);
        f1 = -eta1_global * (1 / ((d - ro_vro - O_SIZE(j))^2)) * (1 + vro / a_max) * nro;
        f2 = ((eta2_global * vro) / (((d - O_SIZE(j)) * a_max) * ((d - O_SIZE(j)) - ro_vro)^2)) * nro_perp;
        
        if (vro > 0) && ((d - O_SIZE(j)) < ro_vro)
            % k = 500;
            % disp('lets see what to do')
        end
        
        if vro <= norm(v_car)
            x1 = x1_b;
            x2 = x2_b;
            x3 = x3_b;
            k = 50;
        else
            x1 = x1_b + 0.9 * vro;
            x2 = x2_b + 0.9 * vro;
            x3 = x3_b + 0.9 * vro;
            k = 50;
        end
        
        if vro > 16
            x1 = 0.9 * 16 + x1_b;
            x2 = 0.9 * 16 + x2_b;
            x3 = 0.9 * 16 + x3_b;
            k = 50;
        end
        
        c_var_attractive = (1 / (1 + 0.1 * exp(-min(d_min) + 10)));
        c_var_nro_perp = k * exp(-(d - O_SIZE(j) - x1)^2 / beta_hill);
        c_var_nro = 1 * exp(-(d - O_SIZE(j) - x2)^2 / beta_hill);
        c_var_reverse = k / (1 + 0.1 * exp(d - O_SIZE(j) - x3));
        
        if c_var_nro <= 0.001
            c_var_nro = 0;
        end
        
        if c_var_nro_perp <= 0.001
            c_var_nro_perp = 0;
        end
        
        if c_var_reverse <= 0.001
            c_var_reverse = 0;
        end
        
        if vro > 0
            f_repulsive = c_var_nro * (f1 + f2) + c_var_nro_perp * f2 + c_var_reverse * f2;
        end
        
        if vro <= 0
            f_repulsive = [0; 0];
        end
        
        if d < O_SIZE(j)
            error('you crashed');
        end
        
        f_rep = f_rep + f_repulsive;
        F_VAR = [F_VAR, f_repulsive];
    end
    
    f_attractive = f_attractive * c_var_attractive;
    F_VAR = [F_VAR, f_attractive];
    NORM_F_VAR = vecnorm(F_VAR);
    f_net_global = f_attractive + f_rep;
    VAR = 0;
    
    for l = 1:numel(OX) + 1
        VAR = VAR + (NORM_F_VAR(l) / sum(NORM_F_VAR)) * (atan2(f_net_global(2), f_net_global(1)) - atan2(F_VAR(2, l), F_VAR(1, l)))^2;
        % VAR=VAR+(atan2(f_net(2),f_net(1))-atan2(F_VAR(2,l),F_VAR(1,l)))^2;
    end
    
    if min(d_min) > 15
        v1 = v_max;
    else
        v1 = v_max * min(d_min) / 15;
    end
    
    if VAR <= 50
        v2 = v_max * (1 - VAR / 50);
    else
        v2 = 0;
    end
    
    L1_MAX = 45;
    L2_MAX = 45;
    var_param = L1_MAX * (1 - VAR / 50);
    dist_param = L2_MAX * ((min(d_min)) / 6 - 1);
    a_plan = 1.9 * tanh(0.1 * (0.6 * dist_param + 0.4 * var_param));
    
    if v <= 2
        v_range = 7;
    end
    
    if v > 2
        v_range = 7 + 1 * v;
    end
    
	%% obstacles in cars frame
    O_car_frame = [];
    OX_selected = [];
    OY_selected = [];
    O_nearest_global = [];

    for j = 1:numel(OX)
        if (((norm(state_car(1:2)' - [OX(j); OY(j)]) - O_SIZE(j)) <= v_range))
            OX_selected = OX(j);
            OY_selected = OY(j);
            temp_vec = -[[OX_selected; OY_selected] - [state_car(1:2)']];
            temp_vec = temp_vec / norm(temp_vec) * (O_SIZE(j));
            tangent_theta = asin(O_SIZE(j) / (norm([[OX_selected; OY_selected] - [state_car(1:2)']])));
            tangent_theta = 90 - rad2deg(tangent_theta);
            O_nearest_global = [O_nearest_global, temp_vec + [OX_selected; OY_selected]];
            
            for k = 5:10:tangent_theta
                rot_matrix_tangent_1 = [cosd(k), -sind(k); sind(k), cosd(k)];
                rot_matrix_tangent_2 = [cosd(-k), -sind(-k); sind(-k), cosd(-k)];
                O_nearest_global = [O_nearest_global, rot_matrix_tangent_1 * temp_vec + [OX_selected; OY_selected], rot_matrix_tangent_2 * temp_vec + [OX_selected; OY_selected]];
            end
        end
    end

    if (~isempty(OX_selected))
        OX_selected = O_nearest_global(1,:);
        OY_selected = O_nearest_global(2,:);
        rot_inv = [cos(-state_car(3)), -sin(-state_car(3)); sin(-state_car(3)), cos(-state_car(3))];
        
        for j = 1:numel(OX_selected)
            O_car_frame = [O_car_frame; (rot_inv * ([OX_selected(j); OY_selected(j)] - (state_car(1:2))'))'];
        end
        
        p_f_car_frame = rot_inv * (p_f - state_car(1:2)');
    
        %% c-space
        lambda = 0;
        
        for j = 1:numel(OX_selected)
            x_f = O_car_frame(j,1);
            y_f = O_car_frame(j,2);    
            
            for n = 1:4
                for k = 0:0.2:1
                    lambda = k;
                    
                    if (n == 1)
                        x_i = -1 + 2 * lambda;
                        y_i = -0.5;
                    end
                    
                    if (n == 2)
                        x_i = 1 - 2 * lambda;
                        y_i = 0.5;
                    end
                    
                    if(n == 3)
                        x_i = -1;
                        y_i = 0.5 - lambda;
                    end
                    
                    if(n == 4)
                        x_i = 1;
                        y_i = -0.5 + lambda;
                    end
                    
                    c_x = [c_x, ((x_f + x_i) * ((y_f^2 - y_i^2) + (x_f^2 - x_i^2)) * ((y_f - y_i)^2 + (x_f - x_i)^2)) / ((y_f - y_i)^4 + 2 * (x_f^2 + x_i^2) * (y_f - y_i)^2 + (x_f^2 - x_i^2)^2)];
                    c_y=[c_y,((y_f-y_i)*((y_f^2-y_i^2)+(x_f^2-x_i^2))*((y_f-y_i)^2+(x_f-x_i)^2))...
                    /((y_f-y_i)^4+2*(x_f^2+x_i^2)*(y_f-y_i)^2+(x_f^2-x_i^2)^2)];
                 end
            end
        end
%% dynamics
p1_x=[];
p2_x=[];
p1_y=[];
p2_y=[];
p3_x=[];
p3_y=[];
p4_x=[];
p4_y=[];
P=[];
for j=1:numel(c_x)
    r=(c_x(j)^2+c_y(j)^2)/(2*c_y(j));
    if(c_y(j)>=0)
        th=atan2(c_x(j),r-c_y(j));
    else
        th=-atan2(c_x(j),c_y(j)-r);
    end
    if(c_y(j)==0)
        l=abs(c_x(j));
        
    else
        l=abs(r*th);
    end
    % translation p1 and p2
    a_v=1.9;
    t=5;
    l_v_max_1=a_v*t^2*(sqrt(1+2*l/(a_v*t^2))-1);
    l_v_max_2=a_v*t^2*(sqrt(1+2*(2*pi*abs(r)-l)/(a_v*t^2))-1);
    if(c_y(j)==0)
        p1_x=[p1_x,sign(c_x(j))*l_v_max_1];
        p1_y=[p1_y,0];
        p2_x=[p2_x,sign(c_x(j))*(2*l-l_v_max_1)];
        %p2_x=[p2_x,sign(c_x(j))*l_v_max_2];
        p2_y=[p2_y,0];
        
    else
        p1_x=[p1_x,r*sin(sign(c_x(j))*l_v_max_1/r)];
        p1_y=[p1_y,r*(1-cos(sign(c_x(j))*l_v_max_1/r))];
        %p2_x=[p2_x,r*sin(sign(c_x(j))*l_v_max_2/r)];
        %p2_y=[p2_y,r*(1-cos(sign(c_x(j))*l_v_max_2/r))];
        p2_x=[p2_x,r*sin(sign(c_x(j))*(2*l-l_v_max_1)/r)];
        p2_y=[p2_y,r*(1-cos(sign(c_x(j))*(2*l-l_v_max_1)/r))];
    end
     rotation p3 and p4
     a_w=1.1;
     t_w_max_1=sign(th)*a_w*t^2*(sqrt(1+2*abs(th)/(a_w*t^2))-1);
     th1=sign(th)*(2*pi-abs(th));
     %th1=atan2(sin(th1),cos(th1));
     t_w_max_2=sign(th1)*a_w*t^2*(sqrt(1+2*abs(th1)/(a_w*t^2))-1);
     if(c_y(j)==0)
        p3_x=[p3_x,c_x(j)];
        p3_y=[p3_y,0];
        p4_x=[p4_x,c_x(j)];
        p4_y=[p4_y,0];
        
    else
        p3_x=[p3_x,r*sin(sign(c_y(j))*t_w_max_1)];
        p3_y=[p3_y,r*(1-cos(sign(c_y(j))*t_w_max_1))];
%         p4_x=[p4_x,r*sin(sign(c_y(j))*t_w_max_2)];
%         p4_y=[p4_y,r*(1-cos(sign(c_y(j))*t_w_max_2))];
        p4_x=[p4_x,r*sin(sign(c_y(j))*(2*th-t_w_max_1))];
        p4_y=[p4_y,r*(1-cos(sign(c_y(j))*(2*th-t_w_max_1)))];

     end
  P_temp=[p1_x(j),p2_x(j),p3_x(j),p4_x(j);p1_y(j),p2_y(j),p3_y(j),p4_y(j)];
  S=(vecnorm([c_x(j);c_y(j)]-P_temp));   
  [b,i]=maxk(S,2);
  S=[P_temp(:,i(1)),P_temp(:,i(2))];
  P=[P,S];
end
c_x=[c_x,p1_x,p2_x];
c_y=[c_y,p1_y,p2_y];
%% ekt
l=[];
alpha=[];
for j=1:numel(c_x)
    if c_y(j)==0
        l=[l,abs(c_x(j))];
    end
    if c_y(j)~=0
        l=[l,abs(((c_x(j)^2+c_y(j)^2)/(2*c_y(j)))*atan2(2*c_x(j)*c_y(j),c_x(j)^2-c_y(j)^2))];
    end
    if c_x(j)>=0
      alpha=[alpha,atan(2*c_y(j)/(c_x(j)^2+c_y(j)^2))];  
    end
    if c_x(j)<0 && c_y(j)~=0
       alpha=[alpha,sign(c_y(j))*pi-atan(2*c_y(j)/(c_x(j)^2+c_y(j)^2))];
    end
    if c_x(j)<0 && c_y(j)==0
       alpha=[alpha,1*pi-atan(2*c_y(j)/(c_x(j)^2+c_y(j)^2))];
    end

end

%% local force planner
f_rep=[0;0];
f_repulsive=[0;0];
for j=1:numel(l)
    nro_local=([l(j)*cos(alpha(j));l(j)*sin(alpha(j))]-state_car(1:2)')/norm([l(j)*cos(alpha(j));l(j)*sin(alpha(j))]-state_car(1:2)');
    vro_local=v_car'*nro_local;
    nro_perp_local_1=[-sin(alpha(j));cos(alpha(j))];
    nro_perp_local_2=-nro_perp_local_1;
    if nro_perp_local_1'*[cos(state_car(3));sin(state_car(3))]>=0
        nro_perp_local=nro_perp_local_1;
    else
        nro_perp_local=nro_perp_local_2;
    end
    
    if((l(j)<=15))
    f_repulsive=-eta1_local*heaviside(-l(j)+7.5)*(1/l(j)-1/15)*(1/(l(j)^2))*[cos(alpha(j));sin(alpha(j))]...
        +eta2_local*heaviside(l(j)-7.5)*((1/l(j)-1/15)*(1/(l(j)^2)))*nro_perp_local;
     %  f_repulsive=-eta1_local*(1/l(j)-1/5.0)*(1/(l(j)^2))*[cos(alpha(j));sin(alpha(j))];
    else
        f_repulsive=[0;0];
    end
    if cos(alpha(j))<=0
        f_repulsive=[0;0];
    end
        
    f_rep=f_rep+f_repulsive;
end
f_net_local=f_rep;
alpha_p=atan2(f_net_local(2),f_net_local(1));
else
    f_net_local=[0;0];
end

%% inverse-ekt and hybrid
rot_matrix=[cos(state_car(3)),-sin(state_car(3));sin(state_car(3)),cos(state_car(3))];
if norm(f_net_local)~=0
f_net_local=rot_matrix*f_net_local;
else
f_net_local=[0;0];
end
% hybrid
differ=real(acos(double(sum(f_net_global.*f_net_local)/((norm(f_net_global))*(norm(f_net_local))))));
differ=abs(atan2(sin(differ),cos(differ)));
mean_local=param_local_g;
if norm(f_net_local)==0
    param_local=0;
else
modulation_local=2*local_var*1/pi*differ-local_var;
param_local=mean_local+modulation_local;
end
if param_local<0
    param_local=0;
end
if param_local>1
    param_local=1;
end
param_global=1-param_local;

if (norm(f_net_global)~=0 && norm(f_net_local)~=0)
f_net=param_global*f_net_global/norm(f_net_global)+param_local*f_net_local/norm(f_net_local);
else
    f_net=param_global*f_net_global+param_local*f_net_local;
end

  gamma=atan2(f_net(2),f_net(1));
  differ_heading=real(acos(sum([cos(state_car(3));sin(state_car(3))].*f_net/(norm(f_net)))));
  differ_heading=abs(atan2(sin(differ_heading),cos(differ_heading)));
  if cos(differ_heading)<0
     f_net=-f_net;
     gamma=atan2(f_net(2),f_net(1));
     v=v-a_plan*t_simulator;
  else
     v=v+a_plan*t_simulator;
  end
  if v>=v_max
     v=v_max;
  end
  if v<=-v_max
     v=-v_max;
  end
%% motion cmds and controller
error=gamma-state_car(3);
    int_error=(int_error+error);
    d_error=(error-prev_error);
    prev_error=error;
    ph=kp*error+ki*int_error+kd*d_error;
    u=atan2(sin(ph),cos(ph));  
    if round(v)~=0
    k=u/v;
    else
    k=0;
    end
    psi_dot=c*(k-sin(state_car(4)));
tspan = (dt):0.005:(dt+t_simulator);
input_car=[v,psi_dot];
cmds = input_car;
[t,y,te,yi,ie] = ode45(@(t,y)derivative(kinematicModel,y,cmds),tspan,state_car,options);
if(te<(dt+t_simulator))
 [t1,y1,te3,ye3,ie3] = ode45(@(t1,y1)derivative(kinematicModel,y1,[v 0]),[te:0.00005:(dt+t_simulator)],y(end,:),options);
 state_car=y1(end,:);
 if(state_car(4) > 0)
 state_car(4)=state_car(4)-0.1;
 else
 state_car(4)=state_car(4)+0.1;   
 end
else
state_car=y(end,:);
end
    cmds_obs=[velocity_obstacle-0*dt , steer_obstacle];
    input_obs=cmds_obs;
    for h=1:n_dynamic_obs
    [t_obs,y_obs] = ode45(@(t_obs,y_obs)derivative(kinematicModel_obs,y_obs,cmds_obs(h,:)),tspan,state_obs(h,:));
    state_obs(h,:)=y_obs(end,:);
    end
    input_target=[v_target steer_target];
    cmds_target=input_target;
    [t_target,y_target] = ode45(@(t,y)derivative(kinematicModel_target,y,cmds_target),tspan,state_target);
    state_target=y_target(end,:);
    p_f=[state_target(1);state_target(2)];
    dt=dt+t_simulator;


%% simulation and plotting 
for h=1:n_dynamic_obs
    ox_d=state_obs(h,1);
    oy_d=state_obs(h,2);
    th = 0:pi/50:2*pi;
    xunit = o_size(h) * cos(th) + ox_d;
    yunit = o_size(h) * sin(th) + oy_d;
    plot(xunit, yunit,'b');
    %filledCircle([ox_d,oy_d],o_size(h),1000,'b');

    %draw_rectangle([ox_d,oy_d],2*o_size(h)*cos(pi/6),2*o_size(h)*sin(pi/6),rad2deg(state_obs(h,3)),[0.1,0.9,0]);
    end
    
    th = 0:pi/50:2*pi;
    xunit = 0.5 * cos(th) + p_f(1);
    yunit = 0.5 * sin(th) + p_f(2);
    plot(xunit, yunit,'b');
    ox=state_obs(:,1)';
    oy=state_obs(:,2)';
    
    for j=1:numel(ox_static)

        th = 0:pi/50:2*pi;
        xunit = o_size_static(j) * cos(th) + ox_static(j);
        yunit = o_size_static(j) * sin(th) + oy_static(j);
        plot(xunit, yunit,'k');
        filledCircle([ox_static(j),oy_static(j)],o_size_static(j),1000,'k');

        hold on
    
    end
    % plot ekt and cspace at each instant, pause and use
%     if ~isempty(OX_selected)
%     for j=1:numel(l)
%        rot_matrix=[cos(state_car(3)),-sin(state_car(3));sin(state_car(3)),cos(state_car(3))];
%        plot_ekt=state_car(1:2)'+rot_matrix*[l(j)*cos(alpha(j));l(j)*sin(alpha(j))];
%        plot_c=state_car(1:2)'+rot_matrix*[c_x(j);c_y(j)];
%        plot(plot_c(1),plot_c(2),'b .'); % c-space
%        %plot(plot_ekt(1),plot_ekt(2),'y .'); % ekt_visualisation
%      hold on 
%     end
%     end 
unicycleTranslations = [state_car(1:2) 0];
unicycleRot = axang2quat([0 0 1 state_car(1,3)]);
%plotTransforms(unicycleTranslations(:,:),unicycleRot(:,:),'MeshFilePath','groundvehicle.stl',"MeshColor","r");
draw_rectangle([unicycleTranslations(1:2)],l_car,b_car,(rad2deg(state_car(3))),[0.0,0,0.1]);
view(0,90);

pause(0.00001);
% plot_force([state_car(1) state_car(2); state_car(1)+f_net(1)/norm(f_net)*v/abs(v) state_car(2)+f_net(2)/norm(f_net)*v/(abs(v))],'g');
% plot_force([state_car(1) state_car(2); state_car(1)+param_local*f_net_local(1)/norm(f_net_local) state_car(2)+f_net_local(2)/norm(f_net_local)],'b');
% plot_force([state_car(1) state_car(2); state_car(1)+param_global*f_net_global(1)/norm(f_net_global) state_car(2)+param_global*f_net_global(2)/norm(f_net_global)],'k');
axis equal;
%x=[x,atan2(f_net(2),f_net(1))-atan2(sin(state_car(3)),cos(state_car(3)))];
%axis([0 50 -50 50]);
%% reset matrices
O_car_frame=[];
c_x=[];
c_y=[];
l=[];
alpha=[];
end