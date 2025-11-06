%% Drone Network Charging & Queuing Simulator
%
% This script simulates a network of N drone charging stations.
% Drones (M) travel between stations, queue if stations are full,
% and charge before traveling again.
%
% It calculates the average time spent in each state:
% 1. Traveling
% 2. Charging
% 3. Waiting (for a charging spot)
%
clear;
clc;
close all;

%% 1. SIMULATION PARAMETERS
fprintf('Setting up simulation parameters...\n');

% --- Graph and Environment ---
N_NODES = 10;               % Number of charging stations (N)
MAP_SIZE = 1000;            % 2D plane size (e.g., 1000x1000 meters)

% --- Drone Population ---
% We set M > N * CAPACITY to ensure waiting times are significant
M_DRONES = 100;             % Total number of drones in the network

% --- Station & Drone Properties ---
BUFFER_CAPACITY = 3;        % Max drones charging at a station
TRAVEL_SPEED = 15;          % Drone speed (e.g., meters/second)
MIN_CHARGE_TIME = 300;      % Min charge time (e.g., seconds)
MAX_CHARGE_TIME = 500;      % Max charge time (e.g., seconds)

% --- Simulation Runtime ---
SIMULATION_TIME = 20000;    % Total seconds to simulate

%% 2. ENVIRONMENT & GRAPH SETUP
% We create N nodes in a 2D plane. The "graph" is the set of nodes.
% Travel is possible between any two nodes (a complete graph).

node_positions = rand(N_NODES, 2) * MAP_SIZE;

% Calculate the all-pairs distance matrix (Euclidean)
% We use nested loops to do this without the 'pdist' function
distance_matrix = zeros(N_NODES, N_NODES); % Pre-allocate the matrix
for i = 1 : N_NODES
    for j = 1 : N_NODES
        % Find the vector from node i to node j
        delta_vec = node_positions(i, :) - node_positions(j, :);
        
        % Calculate Euclidean distance (length of the vector)
        % This is sqrt( (x2-x1)^2 + (y2-y1)^2 )
        distance_matrix(i, j) = sqrt(sum(delta_vec .^ 2));
    end
end

fprintf('Created %d nodes in a %.0fx%.0f area.\n', N_NODES, MAP_SIZE, MAP_SIZE);

%% 3. SIMULATOR INITIALIZATION

% --- Drone State Initialization ---
% We use a struct array to hold the state of every drone
% Initialize an empty struct
drones(M_DRONES) = struct('id', 0, 'state', 'IDLE', ...
    'location', 0, 'destination', 0, 'origin', 0, ...
    'travel_start_time', 0, 'travel_end_time', 0, ...
    'charge_start_time', 0, 'charge_end_time', 0, ...
    'wait_start_time', 0);

for i = 1:M_DRONES
    drones(i).id = i;
    
    % We'll start all drones in a special 'CHARGE_COMPLETE_PENDING' state
    % This will trigger the "find new mission" logic on the first tick.
    drones(i).state = 'CHARGE_COMPLETE_PENDING';
    drones(i).location = randi(N_NODES); % Start at a random node
    drones(i).destination = NaN;
    drones(i).origin = NaN;
    drones(i).travel_start_time = NaN;
    drones(i).travel_end_time = NaN;
    drones(i).charge_start_time = NaN;
    drones(i).charge_end_time = NaN;
    drones(i).wait_start_time = NaN;
end

% --- Station State Initialization ---
% station_buffers(i) = current number of drones charging at node i
station_buffers = zeros(N_NODES, 1);

% waiting_queues{i} = a list (vector) of drone IDs waiting at node i
waiting_queues = cell(N_NODES, 1);
for i = 1:N_NODES
    waiting_queues{i} = []; % Initialize all queues as empty
end

% --- Metric Accumulators ---
total_travel_time = 0;
num_travel_legs = 0;
total_charge_time = 0;
num_charge_sessions = 0;
total_wait_time = 0;
num_wait_sessions = 0;


%% 4. RUN SIMULATION
fprintf('Simulation starting for %d time steps...\n', SIMULATION_TIME);
tic;

for t = 1 : SIMULATION_TIME
    
    % --- 4.1. Update drones based on time (Travel->Arrive, Charge->Done) ---
    % Check for events that finish *at* time t
    
    for i = 1 : M_DRONES
        switch drones(i).state
            case 'TRAVELING'
                if t >= drones(i).travel_end_time
                    % Drone has arrived at its destination
                    drones(i).state = 'ARRIVED_PENDING';
                    drones(i).location = drones(i).destination; % Set new location
                    
                    % Accumulate travel stats
                    travel_duration = drones(i).travel_end_time - drones(i).travel_start_time;
                    total_travel_time = total_travel_time + travel_duration;
                    num_travel_legs = num_travel_legs + 1;
                end
                
            case 'CHARGING'
                if t >= drones(i).charge_end_time
                    % Drone has finished charging
                    drones(i).state = 'CHARGE_COMPLETE_PENDING';
                    
                    % Free up the buffer spot at this station
                    current_node = drones(i).location;
                    station_buffers(current_node) = station_buffers(current_node) - 1;
                    
                    % Accumulate charging stats
                    charge_duration = drones(i).charge_end_time - drones(i).charge_start_time;
                    total_charge_time = total_charge_time + charge_duration;
                    num_charge_sessions = num_charge_sessions + 1;
                end
        end
    end % end drone update loop
    
    
    % --- 4.2. Process state changes (Arrived->Charge/Wait, Done->Travel) ---
    % Handle all drones that just changed state
    
    for i = 1 : M_DRONES
        switch drones(i).state
            case 'ARRIVED_PENDING'
                current_node = drones(i).location;
                if station_buffers(current_node) < BUFFER_CAPACITY
                    % Space available! Start charging.
                    drones(i).state = 'CHARGING';
                    drones(i).charge_start_time = t;
                    % Assign a random charge time
                    charge_duration = randi([MIN_CHARGE_TIME, MAX_CHARGE_TIME]);
                    drones(i).charge_end_time = t + charge_duration;
                    
                    % Occupy buffer spot
                    station_buffers(current_node) = station_buffers(current_node) + 1;
                else
                    % No space. Start waiting.
                    drones(i).state = 'WAITING';
                    drones(i).wait_start_time = t;
                    % Add this drone's ID to the end of the node's queue
                    waiting_queues{current_node} = [waiting_queues{current_node}, i];
                end
                
            case 'CHARGE_COMPLETE_PENDING'
                % Assign a new, random travel mission
                current_node = drones(i).location;
                
                % Find a new destination that is not the current node
                new_destination = current_node;
                while new_destination == current_node
                    new_destination = randi(N_NODES);
                end
                
                % Calculate travel time
                dist = distance_matrix(current_node, new_destination);
                % Ensure travel time is at least 1 time step
                travel_duration = max(1, round(dist / TRAVEL_SPEED)); 
                
                % Set drone to 'TRAVELING'
                drones(i).state = 'TRAVELING';
                drones(i).origin = current_node;
                drones(i).destination = new_destination;
                drones(i).travel_start_time = t;
                drones(i).travel_end_time = t + travel_duration;
                drones(i).location = NaN; % In transit, not at any node
        end
    end % end state processing loop

    
    % --- 4.3. Process waiting queues (Wait->Charge) ---
    % Check all stations for open spots and waiting drones
    
    for n = 1 : N_NODES
        % While there is space AND there are drones waiting at this node
        while station_buffers(n) < BUFFER_CAPACITY && ~isempty(waiting_queues{n})
            
            % Get the first drone from the queue (FIFO)
            drone_id_to_charge = waiting_queues{n}(1);
            waiting_queues{n}(1) = []; % Remove from queue
            
            % Update drone state
            drones(drone_id_to_charge).state = 'CHARGING';
            drones(drone_id_to_charge).charge_start_time = t;
            charge_duration = randi([MIN_CHARGE_TIME, MAX_CHARGE_TIME]);
            drones(drone_id_to_charge).charge_end_time = t + charge_duration;
            
            % Occupy buffer spot
            station_buffers(n) = station_buffers(n) + 1;
            
            % Accumulate waiting stats
            wait_duration = t - drones(drone_id_to_charge).wait_start_time;
            total_wait_time = total_wait_time + wait_duration;
            num_wait_sessions = num_wait_sessions + 1;
        end
    end % end queue processing
    
    % --- Progress Report ---
    if mod(t, 2000) == 0
        fprintf('Time step %d / %d (%.0f%% complete)\n', t, SIMULATION_TIME, (t/SIMULATION_TIME)*100);
    end
    
end % end main simulation loop

sim_duration = toc;
fprintf('Simulation finished in %.2f seconds.\n\n', sim_duration);

%% 5. CALCULATE & DISPLAY RESULTS
fprintf('--- Simulation Results ---\n\n');

% Calculate averages, handling division by zero if an event never happened
if num_travel_legs > 0
    avg_travel_time = total_travel_time / num_travel_legs;
else
    avg_travel_time = 0;
end

if num_charge_sessions > 0
    avg_charge_time = total_charge_time / num_charge_sessions;
else
    avg_charge_time = 0;
end

if num_wait_sessions > 0
    avg_wait_time = total_wait_time / num_wait_sessions;
else
    avg_wait_time = 0;
end

fprintf('Total Simulation Time: %d seconds\n', SIMULATION_TIME);
fprintf('Total Drones: %d\n', M_DRONES);
fprintf('Total Stations: %d\n', N_NODES);
fprintf('Station Capacity: %d\n\n', BUFFER_CAPACITY);

fprintf('--- Averages per Event ---\n');
fprintf('Average Travel Time:   %.2f seconds (based on %d legs)\n', avg_travel_time, num_travel_legs);
fprintf('Average Charging Time: %.2f seconds (based on %d sessions)\n', avg_charge_time, num_charge_sessions);
fprintf('Average Waiting Time:  %.2f seconds (based on %d waits)\n', avg_wait_time, num_wait_sessions);

%% 6. VISUALIZATION
% A simple plot showing the layout of the charging stations

figure;
plot(node_positions(:,1), node_positions(:,2), 'bs', ...
    'MarkerFaceColor', 'b', 'MarkerSize', 10);
title(sprintf('Drone Charging Station Network (N=%d)', N_NODES));
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
grid on;
axis equal;
hold on;

% Label the nodes
for i = 1:N_NODES
    text(node_positions(i,1) + MAP_SIZE*0.01, ...
         node_positions(i,2), ...
         num2str(i), 'Color', 'w', 'FontWeight', 'bold');
end
hold off;