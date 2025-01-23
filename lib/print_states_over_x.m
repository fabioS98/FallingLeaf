%% Function definitions
function print_states_over_x(states,x)
    %check for which index rad to deg conversion applies
    if length(states) == 9     %case x = x
        i_convert = 2:9;
    elseif length(states) == 6     %case x = x
        i_convert = 1:6;
    elseif length(states) == 4 %case x = u
        i_convert = 1:3;
    else                       %other, no conversion is performed
        i_convert = [];
    end
    

    %Print all array states
    i = 1;
    for state = states
        if ismember(i,i_convert)
            fprintf("%s: %f deg\n",state,rad2deg(x(i)));
        else 
            fprintf("%s: %f\n",state,x(i));
        end
        i = i+1;
    end
    fprintf("\n\n")
end