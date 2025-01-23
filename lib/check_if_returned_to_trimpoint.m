function check = check_if_returned_to_trimpoint(TP,out)
    % Checks, if spacecraft is at trim point again at the end of the sim
    %Input:
    % TP: Trim Point
    % out: out signal from the simulation
    %Output:
    % check: true, if returned to tp
    %        false if not returned
    max_delta_from_tp = [deg2rad(1);  %beta 
                         deg2rad(1);  %alpha
                         deg2rad(0.1); %p
                         deg2rad(0.1); %q
                         deg2rad(0.1); %r
                         deg2rad(1)];  %phi

    endState = out.state.data(2:7,1,end);
    trimPoint = TP.op.States.x(2:7);
    
    %check if the end states are withing the defined delta from the trim
    %point
    if all(endState < trimPoint + max_delta_from_tp) ... 
        || all(endState > trimPoint - max_delta_from_tp)
        
        % check if stationary at the end
        if (endState - out.state.data(2:7,1,end-5)) < 0.05 * ones(6,1)
            
            % check for oscillations above 60 deg
            if find(abs(out.state.data(2:6,1,:))>deg2rad(80))
                check = false; % oscillations found
            else
                check = true; % no oscillations found
            end
        else
            check = false; % not stationary
        end
    else
        check = false; % not returned to trim point
    end
  
end