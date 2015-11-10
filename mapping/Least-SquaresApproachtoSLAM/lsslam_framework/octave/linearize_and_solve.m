% performs one iteration of the Gauss-Newton algorithm
% each constraint is linearized and added to the Hessian

function dx = linearize_and_solve(g)

nnz = nnz_of_graph(g);

% allocate the sparse H and the vector b
H = spalloc(length(g.x), length(g.x), nnz);
b = zeros(length(g.x), 1);

needToAddPrior = true;

% compute the addend term to H and b for each of our constraints
disp('linearize and build system');
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the first robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+2);      % the second robot pose

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_pose_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    fromIdx = edge.fromIdx;
    toIdx = edge.toIdx;

    H_00 = A'*edge.information*A;
    H_01 = A'*edge.information*B;
    H_10 = B'*edge.information*A;
    H_11 = B'*edge.information*B;

    H(fromIdx:fromIdx+2,fromIdx:fromIdx+2) = H(fromIdx:fromIdx+2,fromIdx:fromIdx+2) + H_00;
    H(fromIdx:fromIdx+2,toIdx:toIdx+2) = H(fromIdx:fromIdx+2,toIdx:toIdx+2) + H_01;
    H(toIdx:toIdx+2,fromIdx:fromIdx+2) = H(toIdx:toIdx+2,fromIdx:fromIdx+2) + H_10;
    H(toIdx:toIdx+2,toIdx:toIdx+2) = H(toIdx:toIdx+2,toIdx:toIdx+2) + H_11;
    
    b_0 = e'*edge.information*A;
    b_1 = e'*edge.information*B;
    b(fromIdx:fromIdx+2) = b(fromIdx:fromIdx+2) + b_0';
    b(toIdx:toIdx+2) = b(toIdx:toIdx+2) + b_1';
    

    if (needToAddPrior)
      % TODO: add the prior for one pose of this edge
      % This fixes one node to remain at its current location
      H(fromIdx:fromIdx+2, fromIdx:fromIdx+2) = H(fromIdx:fromIdx+2, fromIdx:fromIdx+2) + eye(3,3); %fix one node to (0,0,0)
      %H(1, 1:3) = H(1, 1:3) + ones(1,3); %fix one node to (0,0,0)
      needToAddPrior = false;
    end

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    % edge.fromIdx and edge.toIdx describe the location of
    % the first element of the pose and the landmark in the state vector
    % You should use also this index when updating the elements
    % of the H matrix and the vector b.
    % edge.measurement is the measurement
    % edge.information is the information matrix
    x1 = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    x2 = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    % Computing the error and the Jacobians
    % e the error vector
    % A Jacobian wrt x1
    % B Jacobian wrt x2
    [e, A, B] = linearize_pose_landmark_constraint(x1, x2, edge.measurement);


    % TODO: compute and add the term to H and b
    fromIdx = edge.fromIdx;
    toIdx = edge.toIdx;

    H_00 = A'*edge.information*A;
    H_01 = A'*edge.information*B;

    H_10 = B'*edge.information*A;
    H_11 = B'*edge.information*B;

    H(fromIdx:fromIdx+2,fromIdx:fromIdx+2) = H(fromIdx:fromIdx+2,fromIdx:fromIdx+2) + H_00;
    H(fromIdx:fromIdx+2,toIdx:toIdx+1) = H(fromIdx:fromIdx+2,toIdx:toIdx+1) + H_01;
    H(toIdx:toIdx+1,fromIdx:fromIdx+2) = H(toIdx:toIdx+1,fromIdx:fromIdx+2) + H_10;
    H(toIdx:toIdx+1,toIdx:toIdx+1) = H(toIdx:toIdx+1,toIdx:toIdx+1) + H_11;
    
    b_0 = e'*edge.information*A;
    b_1 = e'*edge.information*B;

    b(fromIdx:fromIdx+2) = b(fromIdx:fromIdx+2) + b_0';
    b(toIdx:toIdx+1) = b(toIdx:toIdx+1) + b_1';


  end
end

disp('solving system');

% TODO: solve the linear system, whereas the solution should be stored in dx
% Remember to use the backslash operator instead of inverting H
    
    dx = -H\b;
end
