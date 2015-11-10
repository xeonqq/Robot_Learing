% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    
    %pose here is in global frame, I assume it is obtained from odometry
    x1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    x2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.

    % here measurement is measuring the next pose j from i directly (i.e. x,y,theta)
    % and it is measured within the local frame (i.e. measure pose j w.r.t pose i)
    % I assume it is obtained by virtual measurement, i.e. by scan matching of landmark and obtain measurement between poses
    e_t = invt(v2t(edge.measurement))*(invt(x1)*x2);
    e_v = t2v(e_t);
    Fx = Fx + e_v'*edge.information*e_v;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.

    % I dont know how the pose-landmark graph is constructed initially.
    % measurement can be done by vision or laser
    x_t = v2t(x);
    l_v = x_t(1:2,1:2)'*(l-x(1:2)); % rotate to robot frame
    e_v = edge.measurement - l_v;
    Fx = Fx + e_v'*edge.information*e_v;


  end

end
