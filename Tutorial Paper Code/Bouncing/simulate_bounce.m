% simulate_system: using ode45, simulate motion of sliding mass
% Diana Frias Franco
% modified from James Zhu
% Carnegie Mellon University Robomechanics Lab

function [state_vec, Xi_vec, time_vec, critical_points, critical_times] = simulate_bounce(x_0,u,t_f)

    params = set_params();
    domain = 1;
    
    new_state = x_0;
    
    t = 0;
    state_vec = [];
    time_vec = [];
    Xi_vec = {}; % for storing saltation matrix calculations

    critical_points = [];
    critical_times = [];
    
    % [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),[t,t_f],new_state); 
    % state_vec = [state_vec; states];

    while t < t_f
        % integrate dynamics using ode45 until an event occurs
        options = odeset('Events', @(t,x)guards(t,x,u,domain,params));
        [time,states] = ode45(@(t,x)flows(t,x,u,domain,params),[t,t_f],new_state,options);

        
        prev_time = time(end);
        end_state = states(end,:)'; % store as column vector
        

        [maxValue, maxIndex] = max(states(:, 1));

        if length(critical_times) > 2
            f = fit(critical_times',critical_points','exp1');
            if (maxValue ) < 0
%             if (f(time(maxIndex)) - maxValue )  < 0
                % then stop integrating
                states = zeros(length(time), 2);
            end
        end


        % find max of that segment
        critical_points = [critical_points, maxValue];
        critical_times = [critical_times, time(maxIndex)];

        



        % store states
        state_vec = [state_vec; states];
        time_vec = [time_vec; time];

        

        %if we've gone through a contact event, calc the saltation matrix
        if prev_time < t_f
            % calculation saltation matrix
            Xi_fun = str2func('calc_Xi_'+string(domain));
            Xi = Xi_fun(end_state,params.parameters);
            Xi_vec{end+1} = Xi;

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