% simulate_system: using ode45, simulate motion of sliding mass
% Diana Frias Franco
% modified from James Zhu
% Carnegie Mellon University Robomechanics Lab

function [state_vec, Xi_1_vec, Xi_2_vec, t_event, t_vec] = simulate_slide(x_0,u,t_f)

    params = set_params();
    domain = 1;
    
    new_state = x_0;
    
    t = 0;
    state_vec = [];
    t_vec = [];
    t_event = [];
    Xi_1_vec = {}; % for storing saltation matrix calculations
    Xi_2_vec = {}; % for storing saltation matrix calculations
    
    % [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),[t,t_f],new_state); 
    % state_vec = [state_vec; states];

    while t < t_f
        % integrate dynamics using ode45 until an event occurs
        options = odeset('Events', @(t,x)guards(t,x,u,domain,params));
        [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),[t,t_f],new_state,options);

        prev_time = time(end);
        end_state = states(end,:)'; % store as column vector

        % store states
        state_vec = [state_vec; states];
        t_vec = [t_vec; time];

        %if we've gone through a contact event, calc the saltation matrix
        if prev_time < t_f
            % calculation saltation matrix
            Xi_1_fun = str2func('calc_Xi_1');
            Xi_1 = Xi_1_fun(end_state,params.parameters);
            Xi_1_vec{end+1} = Xi_1;

            % calc naive update
            Xi_2_fun = str2func('calc_Xi_2');
            Xi_2 = Xi_2_fun(end_state,params.parameters);
            Xi_2_vec{end+1} = Xi_2;

            t_event = [t_event, prev_time];

            % apply reset map after guard is triggered
%             disp("trigger")
            new_state = resets(prev_time, end_state, u, domain, params)';

            % switch domains from 1 to 2, or vice versa
            domain = (domain == 1) + 1;
        else
            new_state = end_state;
        end

        t = prev_time;
    end  

    final_time = prev_time;
    final_state = new_state;
end