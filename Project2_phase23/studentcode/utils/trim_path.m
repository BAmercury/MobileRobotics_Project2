function [ new_path ] = trim_path( path )
%TRIM_PATH Removes waypoints from the path that are along the same line segment
%   The pseudocode for this algorithm, as described in lecture, was:
%   	For each path segment
%			Find the direction of this segment
%			Find the direction of the next segment
%			If the directions are the same
%				Get rid of the point that connects the segments
%   Inputs:
%       path - 2 x n matrix, where each column is a way point
%   Outputs:
%       new_path - 2 x m matrix, where m <= n and each column is a way point from the original path

%     % Only save new points
%     path_new = path(:,1);
%     dir0 = path(:,2) - path(:,1);
%     dir0 = dir0 / norm(dir0);
%     for i = 2:size(path,2)-1
%         dir = path(:,i+1) - path(:,i);
%         dir = dir / norm(dir);
%         if dir' * dir0 < .99
%             path_new = [path_new, path(:,i)];
%             dir0 = dir;
%         end
%     end

keep = true(1, size(path,2)); % this will store the points that we want to keep from the original path

for i = 1 : size(path,2) - 2 % since we are looking at the current segment AND the next segment, meaning two points ahead, we want to stop 2 points before the end of the path
    dir1 = path(:,i+1) - path(:,i); % direction vector for current segment
    dir1 = dir1 / norm(dir1); % make it a unit vector so that it will work for different step sizes
    
    dir2 = path(:,i+2) - path(:,i+1); % direction vector for next segment
    dir2 = dir2 / norm(dir2); % make it a unit vector so that it will work for different step sizes
    
    if all(abs(dir1 - dir2) < 1e-9) % check to make sure that the directions are (almost) the same (in case of tiny rounding errors we don't want to check for exact equality)
        keep(i+1) = false; % we don't want to keep the point that ends the current segment, which is the same point that starts the next segment
    else
%         disp('dirs are NOT equal') % we commented this out since it was used for debugging
    end
end

new_path = path(:,keep); % this will extract only the points (columns) from path that we want to keep

end