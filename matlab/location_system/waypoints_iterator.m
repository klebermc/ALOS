function [ iterator, current_waypoint ] = waypoints_iterator( iterator, current_pos, WAYPOINTS )
%WAYPOINTS_ITERATOR This function will return the next waypoint for the
%robot
%   The desired waypoint is iterated from a matrix, this function will
%   evaluate if the robot has reached a desired waypoint. If the distance
%   from the robot and the waypoint is within a range, the
%   current_waypoint and iterator are updated.
%   current_pos and current_waypoint are in meters [m].

numWP =  size(WAYPOINTS,1);

%just to avoid error when accessing wrong positions at the matrix
if (iterator == 0); iterator=1; end;

if (iterator >= numWP)
    %stops at the last way point
    iterator=numWP;
else
    %Distance from robot to current waypoint
    %dist_robot_wp = sqrt((current_pos(1)-WAYPOINTS(iterator,1))^2 + (current_pos(2)-WAYPOINTS(iterator,2))^2 + (current_pos(3)-WAYPOINTS(iterator,3))^2);
    dist_robot_wp = sqrt((current_pos(1)-WAYPOINTS(iterator,1))^2 + (current_pos(2)-WAYPOINTS(iterator,2))^2 );
    if (dist_robot_wp < 0.25)
        %Reached the current waypoint
        %Setting the waypoint to the next value in the matrix
        iterator = iterator+1;
    end
end
current_waypoint = WAYPOINTS(iterator,:);
end

