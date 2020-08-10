function bool_first_feas = check_first_feasible_new(cur)
global t t_future R target_num;

%% ----------------------------------------------------------------------%%
% This is for just removing branches as much as I can, so the feasiblity is not 
% necessarily exact. i.e.) Target may be outside of the host range for some
% cases. This can be removed when solving problem since infeasible.

temp_t = [t; t_future];

bool_first_feas = 1;
% Check the first feasibility
% (If a drone can cover multi targets at once i.e) d_{t1,t2} < 2 * R_h1
unq = unique(cur(:,1:2)); % e.g) cur(:,1:2) = [1 3; 3 4] =>  unq = [1 3 4]
for i = 1:size(unq,1)
    
    % If a drone covers more than 1 target
    if sum(sum(cur(:,1:2) == unq(i))) > 1       
        % Find the idx of the targets for the drone 
        [row, col] = find(cur(:,1:2) == unq(i)); % e.g) row = [1; 2]; => target 1 and 2 for cur(:,1:2)
        row = [row; row+target_num]; % e.g) [1;2] => [1; 2; 4; 5]; when 3 targets, 4,5 is for future target position.
        % Make all combinations of the targets: nchoosek(row,2) => nC2
        comb_target = nchoosek(row,2);
        % Compute distances between all targets that is covered by a host and if d > 2R, return false
        for j = 1:size(comb_target,1)
            % This is before solving problem, so there is no host position to compute distance between targets and host, 
            % Thus use distance between only targets.
            if sqrt((temp_t(comb_target(j,1),1)-temp_t(comb_target(j,2),1))^2 + (temp_t(comb_target(j,1),2)-temp_t(comb_target(j,2),2))^2) > 2*R
               bool_first_feas = 0;
               return
            end
        end
    end

end