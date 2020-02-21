function c = env_collision_check(x, boundary, blocks, params)
c = false;
% try
R = params.radius;
% catch; keyboard; end
% Check that robot is inside of the environment
if x(1) < boundary(1) + R || x(1) > boundary(4) - R ||...
        x(2) < boundary(2) + R || x(2) > boundary(5) - R || ...
        x(3) < boundary(3) + R || x(3) > boundary(6) - R
    c = true;
    return
end

for i = 1:size(blocks,1)
    % Compute outward facing signed distance to each edge of the block
    sd = [blocks(i,1) - x(1); % neg x side
         blocks(i,2) - x(2);  % neg y side
         blocks(i,3) - x(3);  % neg z
         x(1) - blocks(i,4);  % pos x side
         x(2) - blocks(i,5);  % pos y side
         x(3) - blocks(i,6)]; % pos z side
    
    sd = sd(sd > 0); % only keep values above 0
    if norm(sd) < R % if distance from robot center to obstacle less than robot radius then in collision
        c = true;
        return;
    end
end
