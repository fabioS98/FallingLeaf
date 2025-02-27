function x0 = generate_x0_random_sample(sim_struct, num_samples)
    % This function generates samples for the MC simulation. Entries 2 to
    % 7 from the state vector are all randomized within the hypercube 
    % defined within MC_sim.ranges
    %
    %Input:
    % MC_sim: struct, with all relevant data from the sim
    %Output:
    % returns the initial condition for the sim

    % create latin hypercube matrix
    x0 = zeros(9,num_samples);
    x0(1,:) = ones(1,num_samples) .* sim_struct.TP.op.States.x(1);
    x0(7:9,:) = ones(3,num_samples) .* sim_struct.TP.op.States.x(7:9);
    x0_samples = lhsdesign(num_samples,6)';
    hypercube_length = abs(sim_struct.ranges(:,1)-sim_struct.ranges(:,2));
    for i=1:1:num_samples
        x0(2:7,i) = sim_struct.ranges(:,1) + hypercube_length .* x0_samples(:,i);
    end

end