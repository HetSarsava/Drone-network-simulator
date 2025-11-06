%% Drone Network Live Animation Simulator (Smooth)
%
% This version uses a small time-step (dt) to decouple the simulation
% logic from the animation framerate. This results in a smooth video
% visualization at a specified playback speed.
%
clear;
clc;
close all;

%% 1. SIMULATION PARAMETERS
fprintf('Setting up simulation parameters...\n');

% --- Graph and Environment ---
N_NODES = 25;               % Number of charging stations (N)
MAP_SIZE = 1000;            % 2D plane size (e.g., 1000x1000 meters)
MIN_NODE_DISTANCE = 100;    % Minimum distance between any two nodes

% --- Drone Population ---
M_DRONES = 50;              % Total number of drones (M)

% --- Station & Drone Properties (Scaled for 3-min sim) ---
BUFFER_CAPACITY = 3;        % Max drones charging at a station
TRAVEL_SPEED = 150;         % Drone speed (m/s)
MIN_CHARGE_TIME = 3;        % Min charge time (s)
MAX_CHARGE_TIME = 5;        % Max charge time (s)

% --- Simulation Runtime & Visualization ---
TOTAL_SIM_TIME = 180;       % Total seconds to simulate (3 minutes)
REAL_TIME_SPEEDUP = 3.0;    % Run at 3x speed
TARGET_VISUAL_FPS = 30;     % Target a smooth 30 FPS animation

% --- CALCULATED PARAMETERS ---
% This is our "dt" or the simulation time-step per frame
SIM_TIME_PER_FRAME = REAL_TIME_SPEEDUP / TARGET_VISUAL_FPS;
% (e.g., 3.0 / 30 = 0.1 sim seconds per frame)

% Total number of steps/frames to run
TOTAL_STEPS = round(TOTAL_SIM_TIME / SIM_TIME_PER_FRAME);

% The real-world pause time per frame to hit our target FPS
PAUSE_PER_FRAME = 1.0 / TARGET_VISUAL_FPS;
% (e.g., 1.0 / 30 = 0.0333... real seconds per frame)

VISUALIZATION_PADDING = 100; % Space around the map in the plot


%% 2. ENVIRONMENT & GRAPH SETUP
% (This section is unchanged from the previous version)

node_positions = zeros(N_NODES, 2);
n = 1;
while n <= N_NODES
    new_pos = rand(1, 2) * MAP_SIZE;
    is_valid = true;
    for k = 1 : (n-1)
        dist = sqrt(sum((new_pos - node_positions(k, :)) .^ 2));
        if dist < MIN_NODE_DISTANCE
            is_valid = false;
            break;
        end
    end
    if is_valid
        node_positions(n, :) = new_pos;
        n = n + 1;
    end
end
fprintf('Placed %d nodes with minimum separation.\n', N_NODES);

distance_matrix = zeros(N_NODES, N_NODES);
for i = 1 : N_NODES
    for j = 1 : N_NODES
        delta_vec = node_positions(i, :) - node_positions(j, :);
        distance_matrix(i, j) = sqrt(sum(delta_vec .^ 2));
    end
end

% --- Generate Sparse Graph for Visualization ---
% We will create a graph with N to 2N edges

fprintf('Generating sparse graph for visualization...\n');
node_adj_list = cell(N_NODES, 1); % Adjacency list for checking duplicates
edge_list = zeros(2 * N_NODES, 2); % Pre-allocate for max edges
edge_count = 0;

% 1. Create a Spanning Tree (N-1 edges) to ensure connectivity
for i = 2:N_NODES
    j = randi(i - 1); % Connect to a random node already in the tree
    
    % Add edge (i, j)
    edge_count = edge_count + 1;
    edge_list(edge_count, :) = [i, j];
    node_adj_list{i} = [node_adj_list{i}, j];
    node_adj_list{j} = [node_adj_list{j}, i];
end

% 2. Add random edges to reach our target (N to 2N edges)
% We already have N-1 edges, so we need at least 1 more
target_edge_count = randi([N_NODES, 2*N_NODES]);
max_tries = 100 * N_NODES; % Failsafe to prevent infinite loop
tries = 0;

while edge_count < target_edge_count && tries < max_tries
    % Pick two random nodes
    u = randi(N_NODES);
    v = randi(N_NODES);
    
    % Check if edge is valid (not a self-loop and not a duplicate)
    if u ~= v && ~ismember(v, node_adj_list{u})
        % Add edge (u, v)
        edge_count = edge_count + 1;
        edge_list(edge_count, :) = [u, v];
        node_adj_list{u} = [node_adj_list{u}, v];
        node_adj_list{v} = [node_adj_list{v}, u];
    end
    tries = tries + 1;
end

% Trim the final edge list to the actual number of edges
edge_list = edge_list(1:edge_count, :);
fprintf('Generated %d edges for the sparse graph.\n', edge_count);

% --- (Your existing code for generating edge_list) ---
% ...
% edge_list = edge_list(1:edge_count, :);
fprintf('Generated %d edges for the sparse graph.\n', edge_count);

% --- NEW: Create Weighted Graph Object for Pathfinding ---
% Create a sparse adjacency matrix with 'Inf' (no connection)
sparse_adj_matrix = ones(N_NODES, N_NODES) * Inf;
for k = 1:edge_count
    u = edge_list(k, 1);
    v = edge_list(k, 2);
    
    % Get the distance (weight) from the full distance_matrix
    dist = distance_matrix(u, v);
    
    % Add the edge in both directions
    sparse_adj_matrix(u, v) = dist;
    sparse_adj_matrix(v, u) = dist;
end

% Create the MATLAB graph object
G = graph(sparse_adj_matrix, 'omitselfloops');

% Failsafe: Check if graph is disconnected (our tree should prevent this)
bins = conncomp(G);
if max(bins) > 1
    warning('Graph is disconnected! Some nodes may be unreachable.');
end

%% 3. SIMULATOR INITIALIZATION
% (This section is unchanged)

% --- Drone State Initialization ---
drones(M_DRONES) = struct('id', 0, 'state', 'IDLE', ...
    'location', 0, 'destination', 0, 'origin', 0, ...
    'travel_start_time', 0, 'travel_end_time', 0, ...
    'charge_start_time', 0, 'charge_end_time', 0, ...
    'wait_start_time', 0, ...
    'path', [], 'current_leg_index', 0); % <-- NEW FIELDS

for i = 1:M_DRONES
    drones(i).id = i;
    drones(i).state = 'CHARGE_COMPLETE_PENDING';
    drones(i).location = randi(N_NODES);
end

station_buffers = zeros(N_NODES, 1);
waiting_queues = cell(N_NODES, 1);
for i = 1:N_NODES
    waiting_queues{i} = [];
end

% --- Metric Accumulators ---
total_travel_time = 0;
num_travel_legs = 0;
total_charge_time = 0;
num_charge_sessions = 0;
total_wait_time = 0;
num_wait_sessions = 0;

% --- Visualization Setup ---
fprintf('Simulation starting... Opening animation window.\n');
h_fig = figure;
set(h_fig, 'Position', [100, 100, 800, 700]);
h_ax = gca;
axis(h_ax, [-VISUALIZATION_PADDING, MAP_SIZE + VISUALIZATION_PADDING, ...
             -VISUALIZATION_PADDING, MAP_SIZE + VISUALIZATION_PADDING]);
axis equal;
grid on;


%% 4. RUN SIMULATION
tic; % Start the total simulation timer

current_time = 0; % Our simulation clock starts at 0

for step = 1 : TOTAL_STEPS
    
    frame_timer = tic; % Start timer for this frame
    
    % Advance the simulation clock
    current_time = step * SIM_TIME_PER_FRAME;
    
    % --- 4.1. Update drones based on time (Travel->Arrive, Charge->Done) ---
    % This logic is now based on 'current_time' (a float)
    for i = 1 : M_DRONES
        switch drones(i).state
            case 'TRAVELING'
                if current_time >= drones(i).travel_end_time
                    % This leg of the journey is complete
                    drones(i).state = 'LEG_COMPLETE_PENDING';
                    
                    % Set location to the node just arrived at
                    leg_index = drones(i).current_leg_index;
                    drones(i).location = drones(i).path(leg_index + 1);
                    
                    % Accumulate travel stats for this leg
                    travel_duration = drones(i).travel_end_time - drones(i).travel_start_time;
                    total_travel_time = total_travel_time + travel_duration;
                    num_travel_legs = num_travel_legs + 1;
                end
                
            case 'CHARGING'
                if current_time >= drones(i).charge_end_time
                    drones(i).state = 'CHARGE_COMPLETE_PENDING';
                    current_node = drones(i).location;
                    station_buffers(current_node) = station_buffers(current_node) - 1;
                    charge_duration = drones(i).charge_end_time - drones(i).charge_start_time;
                    total_charge_time = total_charge_time + charge_duration;
                    num_charge_sessions = num_charge_sessions + 1;
                end
        end
    end
    
    
    % --- 4.2. Process state changes (Arrived->Charge/Wait, Done->Travel) ---
    % --- 4.2. Process state changes (Arrived->Charge/Wait, Done->Travel) ---
    for i = 1 : M_DRONES
        switch drones(i).state
            case 'ARRIVED_PENDING'
                current_node = drones(i).location;
                if station_buffers(current_node) < BUFFER_CAPACITY
                    drones(i).state = 'CHARGING';
                    drones(i).charge_start_time = current_time;
                    charge_duration = randi([MIN_CHARGE_TIME, MAX_CHARGE_TIME]);
                    drones(i).charge_end_time = current_time + charge_duration;
                    station_buffers(current_node) = station_buffers(current_node) + 1;
                else
                    drones(i).state = 'WAITING';
                    drones(i).wait_start_time = current_time;
                    waiting_queues{current_node} = [waiting_queues{current_node}, i];
                end
                
            case 'LEG_COMPLETE_PENDING'
                % Drone has finished one leg of its path
                leg_index = drones(i).current_leg_index;
                current_path = drones(i).path;
                
                if leg_index + 1 == length(current_path)
                    % This was the FINAL leg. Arrive at destination.
                    drones(i).state = 'ARRIVED_PENDING';
                    drones(i).path = [];
                    drones(i).current_leg_index = 0;
                else
                    % Not the final leg. Start the NEXT leg.
                    drones(i).state = 'TRAVELING';
                    drones(i).current_leg_index = leg_index + 1;
                    
                    % Set up the new leg
                    origin_node = drones(i).path(drones(i).current_leg_index);
                    dest_node = drones(i).path(drones(i).current_leg_index + 1);
                    
                    dist = sparse_adj_matrix(origin_node, dest_node);
                    travel_duration = max(SIM_TIME_PER_FRAME, (dist / TRAVEL_SPEED));
                    
                    drones(i).origin = origin_node;
                    drones(i).destination = dest_node;
                    drones(i).travel_start_time = current_time;
                    drones(i).travel_end_time = current_time + travel_duration;
                    drones(i).location = NaN; % In transit
                end
                
            case 'CHARGE_COMPLETE_PENDING'
                % Assign a new, random travel mission
                current_node = drones(i).location;
                
                % Find a new final destination
                new_destination = current_node;
                while new_destination == current_node
                    new_destination = randi(N_NODES);
                end
                
                % Find the shortest path using our graph G
                [path, path_dist] = shortestpath(G, current_node, new_destination);
                
                if ~isempty(path) && path_dist > 0 && path_dist ~= Inf
                    % A valid path was found! Start the first leg.
                    drones(i).path = path;
                    drones(i).current_leg_index = 1;
                    drones(i).state = 'TRAVELING';
                    
                    % Get first leg details
                    origin_node = drones(i).path(1);
                    dest_node = drones(i).path(2);
                    
                    % Get leg distance from the adjacency matrix
                    dist = sparse_adj_matrix(origin_node, dest_node);
                    travel_duration = max(SIM_TIME_PER_FRAME, (dist / TRAVEL_SPEED));
                    
                    drones(i).origin = origin_node;
                    drones(i).destination = dest_node;
                    drones(i).travel_start_time = current_time;
                    drones(i).travel_end_time = current_time + travel_duration;
                    drones(i).location = NaN; % In transit
                    
                else
                    % No path found (e.g., disconnected graph or same node)
                    % Stay at the station and try again next time.
                    drones(i).state = 'CHARGE_COMPLETE_PENDING';
                    drones(i).location = current_node;
                end
        end
    end

    
    % --- 4.3. Process waiting queues (Wait->Charge) ---
    for n = 1 : N_NODES
        while station_buffers(n) < BUFFER_CAPACITY && ~isempty(waiting_queues{n})
            drone_id_to_charge = waiting_queues{n}(1);
            waiting_queues{n}(1) = [];
            
            drones(drone_id_to_charge).state = 'CHARGING';
            drones(drone_id_to_charge).charge_start_time = current_time;
            charge_duration = randi([MIN_CHARGE_TIME, MAX_CHARGE_TIME]);
            drones(drone_id_to_charge).charge_end_time = current_time + charge_duration;
            station_buffers(n) = station_buffers(n) + 1;
            
            wait_duration = current_time - drones(drone_id_to_charge).wait_start_time;
            total_wait_time = total_wait_time + wait_duration;
            num_wait_sessions = num_wait_sessions + 1;
        end
    end
    
    % --- 4.4. VISUALIZATION ---
    
    cla(h_ax);
    hold(h_ax, 'on');
    
    % Plot all graph edges (from our sparse list)
    edge_color = [0.8 0.8 0.8];
    for k = 1:size(edge_list, 1) % Loop through our pre-made edge list
        % Get the two nodes for this edge
        u = edge_list(k, 1);
        v = edge_list(k, 2);
        
        % Get their positions
        p1 = node_positions(u, :);
        p2 = node_positions(v, :);
        
        % Plot the single dotted line
        plot(h_ax, [p1(1), p2(1)], [p1(2), p2(2)], ':', 'Color', edge_color);
    end
    
    % Plot all nodes
    plot(h_ax, node_positions(:,1), node_positions(:,2), 'bs', ...
        'MarkerFaceColor', 'b', 'MarkerSize', 10);
    
    % Plot all drones
    for i = 1 : M_DRONES
        current_pos = [NaN, NaN];
        
        if strcmp(drones(i).state, 'TRAVELING')
            % Interpolate position
            start_pos = node_positions(drones(i).origin, :);
            end_pos = node_positions(drones(i).destination, :);
            
            duration = drones(i).travel_end_time - drones(i).travel_start_time;
            % Use 'current_time' for a smooth interpolation
            elapsed = current_time - drones(i).travel_start_time;
            fraction = max(0, min(1, elapsed / duration));
            
            current_pos = start_pos + fraction * (end_pos - start_pos);
            
        elseif ~isnan(drones(i).location)
            % Drone is at a station
            current_pos = node_positions(drones(i).location, :);
            current_pos = current_pos + (rand(1,2) - 0.5) * (MAP_SIZE / 50);
        end
        
        if ~isnan(current_pos(1))
            plot(h_ax, current_pos(1), current_pos(2), 'r^', ...
                'MarkerFaceColor', 'r', 'MarkerSize', 6);
        end
    end
    
    % Update title with the new float-based time
    title(h_ax, sprintf('Drone Network (%.0fx Speed) | Time: %.1fs / %ds', ...
          REAL_TIME_SPEEDUP, current_time, TOTAL_SIM_TIME));
    hold(h_ax, 'off');
    
    % --- Pause logic for smooth FPS ---
    elapsed_frame_time = toc(frame_timer);
    pause_duration = max(0, PAUSE_PER_FRAME - elapsed_frame_time);
    pause(pause_duration);
    
    drawnow;
    
    if ~ishandle(h_fig)
        fprintf('\nAnimation window closed. Simulation stopped.\n');
        break;
    end
    
end % end main simulation loop

sim_duration = toc;
fprintf('Simulation finished.\n');
fprintf('Target real-time runtime: %.2f seconds.\n', TOTAL_SIM_TIME / REAL_TIME_SPEEDUP);
fprintf('Actual real-time elapsed: %.2f seconds.\n\n', sim_duration);

%% 5. CALCULATE & DISPLAY RESULTS
% (This section is unchanged)
fprintf('--- Simulation Results ---\n\n');

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

fprintf('Total Simulated Time: %d seconds\n', TOTAL_SIM_TIME);
fprintf('Total Drones: %d\n', M_DRONES);
fprintf('Total Stations: %d\n', N_NODES);
fprintf('Station Capacity: %d\n\n', BUFFER_CAPACITY);

fprintf('--- Averages per Event ---\n');
fprintf('Average Travel Time:   %.2f seconds (based on %d legs)\n', avg_travel_time, num_travel_legs);
fprintf('Average Charging Time: %.2f seconds (based on %d sessions)\n', avg_charge_time, num_charge_sessions);
fprintf('Average Waiting Time:  %.2f seconds (based on %d waits)\n', avg_wait_time, num_wait_sessions);