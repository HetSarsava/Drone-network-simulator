function drone_simulation()
    close all;
    clear;
    clc;

    % --- Configuration ---
    N = 8;        % Number of Stations
    D = 5;        % Number of Drones
    TIME_LIMIT = 60; % Hard-coded 1 minute simulation time
    dt = 0.05;
    
    % Stats Tracking
    total_wait_seconds = 0;
    total_wait_events = 0;
    
    % --- 1. Generate Stations & Graph ---
    sim_range = 15; 
    stations = zeros(N, 2);
    min_dist = 8; 
    max_attempts = 1000;
    
    for i = 1:N
        valid = false; attempts = 0;
        while ~valid && attempts < max_attempts
            attempts = attempts + 1;
            pos = (rand(1, 2) - 0.5) * 2 * (sim_range - 2); 
            valid = true;
            for j = 1:i-1
                if norm(pos - stations(j,:)) < min_dist
                    valid = false; break;
                end
            end
            if valid, stations(i, :) = pos; end
        end
        if ~valid, stations(i, :) = pos; end
    end

    % Delaunay Graph
    tri = delaunay(stations(:,1), stations(:,2));
    adjacency = cell(1, N);
    unique_edges = [];
    for i = 1:size(tri, 1)
        pairs = [tri(i,1) tri(i,2); tri(i,2) tri(i,3); tri(i,3) tri(i,1)];
        for k = 1:3
            u = pairs(k, 1); v = pairs(k, 2);
            if ~ismember(v, adjacency{u}), adjacency{u} = [adjacency{u}, v]; end
            if ~ismember(u, adjacency{v}), adjacency{v} = [adjacency{v}, u]; end
            edge = sort([u, v]);
            unique_edges = [unique_edges; edge];
        end
    end
    unique_edges = unique(unique_edges, 'rows');

    % --- 2. Simulation State Initialization ---
    station_h = 3;
    cruise_h = 10; 
    station_width = 4.0; 
    charging_time = 3.0;
    flight_speed = 5.0; 
    pad_dist = station_width * 0.3;
    
    % Pad Occupancy Grid: (Station x Pad) -> Drone ID (0 if empty)
    pad_occupancy = zeros(N, 6);
    
    % Initialize Drones
    drones = struct('pos', {}, 'target', {}, 'state', {}, 'station', {}, ...
                    'next_station', {}, 'pad', {}, 'timer', {}, 'color', {}, 'yaw', {});
    
    colors = hsv(D); 
    
    for i = 1:D
        placed = false;
        while ~placed
            s_idx = randi(N);
            p_idx = randi(6);
            if pad_occupancy(s_idx, p_idx) == 0
                pad_occupancy(s_idx, p_idx) = i;
                
                angle = (p_idx-1) * pi/3;
                px = stations(s_idx, 1) + pad_dist * cos(angle);
                py = stations(s_idx, 2) + pad_dist * sin(angle);
                
                drones(i).pos = [px, py, station_h];
                drones(i).target = [px, py, station_h];
                
                % Check initial capacity for this station
                active_chargers = 0;
                for k = 1:i-1
                    if drones(k).station == s_idx && strcmp(drones(k).state, 'CHARGING')
                        active_chargers = active_chargers + 1;
                    end
                end
                
                if active_chargers < 3
                    drones(i).state = 'CHARGING';
                    drones(i).timer = rand() * charging_time; 
                else
                    drones(i).state = 'WAITING';
                    drones(i).timer = 0;
                    total_wait_events = total_wait_events + 1; 
                end
                
                drones(i).station = s_idx;
                drones(i).next_station = -1;
                drones(i).pad = p_idx;
                drones(i).color = colors(i, :); 
                drones(i).yaw = 0;
                placed = true;
            end
        end
    end

    % --- 3. Visualization Setup ---
    fig = figure('Name', 'Multi-Drone Hub Simulation', 'Color', 'k'); 
    axis_limit = sim_range + 3;
    axis([-axis_limit axis_limit -axis_limit axis_limit 0 16]);
    grid on; axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2); 
    view(3); hold on;

    % --- UI Controls for Adjustment ---
    hSlider = uicontrol('Style', 'slider', 'Min', 0.0, 'Max', 0.1, 'Value', 0.01, ...
        'Position', [20 20 200 20]);
    uicontrol('Style','text', 'Position',[20 45 200 20], ...
        'String','Playback Speed (Delay)', ...
        'BackgroundColor', 'k', 'ForegroundColor', 'w');

    % Draw Graph
    for i = 1:size(unique_edges, 1)
        u = unique_edges(i, 1); v = unique_edges(i, 2);
        p1 = stations(u, :); p2 = stations(v, :);
        plot3([p1(1) p2(1)], [p1(2) p2(2)], [station_h/2 station_h/2], '--', 'Color', [0.3 0.3 0.3]);
        plot3([p1(1) p2(1)], [p1(2) p2(2)], [cruise_h cruise_h], ':', 'Color', 'w', 'LineWidth', 0.8);
    end
    for i = 1:N
        plot3([stations(i,1) stations(i,1)], [stations(i,2) stations(i,2)], [station_h cruise_h], ':', 'Color', [0.4 0.4 0.4]);
        draw_station_building(stations(i, :), station_width, station_h, sprintf('HUB-%d', i));
    end

    % Create Graphics Objects for Drones
    drone_gfx = [];
    for i = 1:D
        drone_gfx(i).body = plot3(0,0,0, '-', 'Color', 'w', 'LineWidth', 2);
        drone_gfx(i).rotors = plot3(0,0,0, '.', 'Color', 'c'); 
        drone_gfx(i).fov = plot3(0,0,0, '-', 'Color', drones(i).color, 'LineWidth', 0.5);
        drone_gfx(i).text = text(0,0,0, '', 'Color', 'w', 'FontSize', 8, 'HorizontalAlignment', 'center');
    end

    % --- 4. Main Simulation Loop ---
    sim_time_elapsed = 0; % Use simulation time, not wall clock
    
    while ishandle(fig)
        
        % Check Timer based on SIMULATION steps
        sim_time_elapsed = sim_time_elapsed + dt;
        remaining_time = TIME_LIMIT - sim_time_elapsed;
        
        title(gca, sprintf('Sim Time: %.1f s / %.1f s', sim_time_elapsed, TIME_LIMIT), 'Color', 'w', 'FontSize', 12);
        
        if remaining_time <= 0
            break; % Terminate loop when SIMULATION time ends
        end
        
        for i = 1:D
            d = drones(i);
            reached = norm(d.pos - d.target) < 0.1;
            
            % Update Waiting Stats
            if strcmp(d.state, 'WAITING')
                total_wait_seconds = total_wait_seconds + dt;
            end
            
            switch d.state
                case 'CHARGING'
                    d.timer = d.timer - dt;
                    if d.timer <= 0
                        % --- CHARGING COMPLETE ---
                        
                        % 1. Release Spot logic
                        current_station_idx = d.station;
                        
                        % Find all WAITING drones at this station
                        waiting_indices = [];
                        for k = 1:D
                            if drones(k).station == current_station_idx && strcmp(drones(k).state, 'WAITING')
                                waiting_indices = [waiting_indices, k];
                            end
                        end
                        
                        % If anyone is waiting, promote the one with the lowest Pad Index
                        if ~isempty(waiting_indices)
                            min_pad = 7;
                            promote_idx = -1;
                            
                            for k = waiting_indices
                                if drones(k).pad < min_pad
                                    min_pad = drones(k).pad;
                                    promote_idx = k;
                                end
                            end
                            
                            % Promote
                            if promote_idx > 0
                                drones(promote_idx).state = 'CHARGING';
                                drones(promote_idx).timer = charging_time;
                            end
                        end
                        
                        % 2. Pick next destination
                        neighbors = adjacency{d.station};
                        if ~isempty(neighbors)
                            d.next_station = neighbors(randi(length(neighbors)));
                            pad_occupancy(d.station, d.pad) = 0; 
                            d.pad = 0;
                            d.target = [d.pos(1), d.pos(2), cruise_h];
                            d.state = 'TAKEOFF';
                        else
                            d.timer = 1.0; 
                        end
                    end
                    
                case 'TAKEOFF'
                    if reached
                        s_pos = stations(d.station, :);
                        d.target = [s_pos(1), s_pos(2), cruise_h];
                        d.state = 'TRANSIT_TO_CENTER';
                    end
                    
                case 'TRANSIT_TO_CENTER'
                    if reached
                        ns_pos = stations(d.next_station, :);
                        d.target = [ns_pos(1), ns_pos(2), cruise_h];
                        d.state = 'CRUISE';
                    end
                    
                case 'CRUISE'
                    if reached
                        d.station = d.next_station;
                        free_pad = 0;
                        for p = 1:6
                            if pad_occupancy(d.station, p) == 0
                                free_pad = p;
                                break;
                            end
                        end
                        
                        if free_pad > 0
                            pad_occupancy(d.station, free_pad) = i;
                            d.pad = free_pad;
                            angle = (free_pad-1) * pi/3;
                            px = stations(d.station, 1) + pad_dist * cos(angle);
                            py = stations(d.station, 2) + pad_dist * sin(angle);
                            d.target = [px, py, cruise_h];
                            d.state = 'ALIGN_PAD';
                        else
                            d.state = 'HOVERING';
                            d.timer = 0.5; 
                            offset = (rand(1,3)-0.5)*1.5; 
                            offset(3) = 0; 
                            s_pos = stations(d.station, :);
                            d.target = [s_pos(1), s_pos(2), cruise_h] + offset; 
                        end
                    end
                    
                case 'HOVERING'
                    d.timer = d.timer - dt;
                    if d.timer <= 0
                        free_pad = 0;
                        for p = 1:6
                            if pad_occupancy(d.station, p) == 0
                                free_pad = p;
                                break;
                            end
                        end
                        
                        if free_pad > 0
                            pad_occupancy(d.station, free_pad) = i;
                            d.pad = free_pad;
                            angle = (free_pad-1) * pi/3;
                            px = stations(d.station, 1) + pad_dist * cos(angle);
                            py = stations(d.station, 2) + pad_dist * sin(angle);
                            d.target = [px, py, cruise_h];
                            d.state = 'ALIGN_PAD';
                        else
                            d.timer = 0.5; 
                        end
                    end
                    
                case 'ALIGN_PAD'
                    if reached
                        d.target = [d.pos(1), d.pos(2), station_h];
                        d.state = 'LANDING';
                    end
                    
                case 'LANDING'
                    if reached
                        % Check number of currently charging drones at this station
                        active_chargers = 0;
                        for k = 1:D
                            if drones(k).station == d.station && strcmp(drones(k).state, 'CHARGING')
                                active_chargers = active_chargers + 1;
                            end
                        end
                        
                        if active_chargers < 3
                            d.state = 'CHARGING';
                            d.timer = charging_time;
                        else
                            d.state = 'WAITING';
                            d.timer = 0; 
                            total_wait_events = total_wait_events + 1; 
                        end
                    end
                    
                case 'WAITING'
                    % Waiting logic
            end
            
            % Physics Move
            diff = d.target - d.pos;
            dist = norm(diff);
            if dist > 0
                move_dist = min(dist, flight_speed * dt);
                direction = diff / dist;
                d.pos = d.pos + direction * move_dist;
                
                if norm(direction(1:2)) > 0.1
                    target_yaw = atan2(direction(2), direction(1));
                    d.yaw = target_yaw; 
                end
            end
            
            drones(i) = d;
            update_drone_gfx(drone_gfx(i), d, station_h);
        end
        
        drawnow;

        if ~ishandle(hSlider)
            break;
        end

        delay_val = get(hSlider, 'Value');
        pause(delay_val); 
    end
    
    % --- End of Simulation Stats ---
    if ishandle(fig)
        avg_wait_time = 0;
        if total_wait_events > 0
            avg_wait_time = total_wait_seconds / total_wait_events;
        end
        
        msg = sprintf(['Simulation Completed (Sim Time Reached)\n\n', ...
                       'Number of Stations: %d\n', ...
                       'Number of Drones: %d\n', ...
                       'Average Waiting Time: %.2f seconds'], ...
                       N, D, avg_wait_time);
        
        msgbox(msg, 'Simulation Stats');
        title(gca, 'Simulation Ended', 'Color', 'r');
    end
end

function update_drone_gfx(gfx, d, station_h)
    arm_len = 0.5;
    cx = d.pos(1); cy = d.pos(2); cz = d.pos(3);
    
    status_str = d.state;
    status_col = 'w';
    if strcmp(d.state, 'CHARGING'), status_col = 'g'; end
    if strcmp(d.state, 'WAITING'), status_col = 'y'; end 
    if strcmp(d.state, 'HOVERING'), status_col = [1 0.5 0]; end 
    
    set(gfx.text, 'Position', [cx, cy, cz+1.5], 'String', status_str, 'Color', status_col);
    
    R = [cos(d.yaw) -sin(d.yaw) 0; sin(d.yaw) cos(d.yaw) 0; 0 0 1];
    arms_local = [arm_len 0 0; -arm_len 0 0; 0 arm_len 0; 0 -arm_len 0]';
    arms_world = R * arms_local;
    
    bx = [cx + arms_world(1, 1), cx + arms_world(1, 2), nan, cx + arms_world(1, 3), cx + arms_world(1, 4)];
    by = [cy + arms_world(2, 1), cy + arms_world(2, 2), nan, cy + arms_world(2, 3), cy + arms_world(2, 4)];
    bz = [cz + arms_world(3, 1), cz + arms_world(3, 2), nan, cz + arms_world(3, 3), cz + arms_world(3, 4)];
    set(gfx.body, 'XData', bx, 'YData', by, 'ZData', bz);
    
    fov_w = 0.8;
    fx = [cx, cx-fov_w, cx+fov_w, cx, cx+fov_w, cx-fov_w, cx, cx-fov_w];
    fy = [cy, cy-fov_w, cy-fov_w, cy, cy+fov_w, cy+fov_w, cy, cy-fov_w];
    fz = [cz, cz-1, cz-1, cz, cz-1, cz-1, cz, cz-1];
    set(gfx.fov, 'XData', fx, 'YData', fy, 'ZData', fz);
end

function draw_station_building(pos, w, h, name)
    x = pos(1); y = pos(2);
    
    theta = linspace(0, 2*pi, 7);
    r_build = w * 0.5;
    xb = x + r_build * cos(theta);
    yb = y + r_build * sin(theta);
    zb_top = h * ones(size(xb));
    
    patch(xb, yb, zb_top, [0.15 0.15 0.2], 'EdgeColor', [0.5 0.5 0.5]); 
    
    for i = 1:6
        patch([xb(i) xb(i+1) xb(i+1) xb(i)], [yb(i) yb(i+1) yb(i+1) yb(i)], ...
              [0 0 h h], [0.2 0.2 0.25], 'EdgeColor', 'none');
    end
    
    pad_dist = w * 0.3; pad_r = w * 0.08;
    for i = 1:6
        angle = (i-1) * pi/3;
        px = x + pad_dist * cos(angle);
        py = y + pad_dist * sin(angle);
        
        t = 0:0.8:2*pi;
        
        if i <= 3
            p_color = [0.2 0.8 0.2]; 
        else
            p_color = [0.8 0.7 0.1]; 
        end
        
        fill3(px + pad_r*cos(t), py + pad_r*sin(t), (h+0.05)*ones(size(t)), ...
              p_color, 'EdgeColor', 'none');
        text(px, py, h + 0.1, num2str(i), 'Color', 'k', 'FontSize', 6, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    text(x, y, h + 1.5, name, 'HorizontalAlignment', 'center', 'Color', 'w', 'FontSize', 9);
end