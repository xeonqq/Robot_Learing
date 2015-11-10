% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  fx_x = -cos(x(3));
  fy_x = sin(x(3));

  fx_y = -sin(x(3));
  fy_y = -cos(x(3));
  
  fx_theta = -sin(x(3))*(l(1)-x(1)) + cos(x(3))*(l(2)-x(2));
  fy_theta = -cos(x(3))*(l(1)-x(1)) - sin(x(3))*(l(2)-x(2));

  A = [ fx_x, fx_y, fx_theta;
  	fy_x, fy_y, fy_theta];

  gx_x = - fx_x;
  gx_y = - fx_y;
  gy_x = - fy_x;
  gy_y = - fy_y;

  B = [ gx_x, gx_y;
  	gy_x, gy_y];

  x_t = v2t(x);
  l_v = x_t(1:2,1:2)'*(l-x(1:2)); % rotate to robot frame
  e = l_v - z;
  

end;
