function saltation_matrix = compute_saltation_matrix(t, pre_event_state, inputs, dt, parameters, pre_mode, post_mode, dynamics_dict, resets_dict, guards_dict, post_event_state)
    % This function computes the saltation matrix based on pre and post-event states.
    if nargin < 11 || isempty(post_event_state)
        post_event_state = resets_dict{pre_mode}{post_mode}.r(pre_event_state, inputs, dt, parameters);
    end
    
    DxR =  resets_dict.(pre_mode).(post_mode).R(pre_event_state, inputs, dt, parameters);
    DxG = guards_dict.(pre_mode).(post_mode).G(pre_event_state, inputs, dt, parameters);
    DtG = guards_dict.(pre_mode).(post_mode).Gt(t, pre_event_state, inputs, dt, parameters);
    f_pre = dynamics_dict.(pre_mode).f_cont(pre_event_state, inputs, dt, parameters);
    f_post = dynamics_dict.(post_mode).f_cont(post_event_state, inputs, dt, parameters);
    
    saltation_matrix = DxR + ((f_post - DxR * f_pre) * DxG) / (DtG + DxG * f_pre);
end