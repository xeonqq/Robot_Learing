% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  % Jacobian of x1
  fx_x = -cos(z(3))*cos(x1(3)) + sin(z(3))*sin(x1(3));
  fx_y = -cos(z(3))*sin(x1(3)) - sin(z(3))*cos(x1(3));
  fx_theta = fx_y*(x2(1)-x1(1)) + (-fx_x)*(x2(2)-x1(2));

  fy_x = -fx_y;  %sin(z.theta)*cos(x1.theta) + cos(z.theta)*sin(x1.theta) 
  fy_y = fx_x;  %sin(z.theta)*sin(x1.theta) - cos(z.theta)*cos(x1.theta) 
  fy_theta = fy_y*(x2(1)-x1(1)) + (-fy_x)*(x2(2)-x1(2));
  A = [ fx_x, fx_y, fx_theta;
  	fy_x, fy_y, fy_theta;
  	0, 0, -1];

  % Jacobian of x2
  gx_x = -fx_x; %cos(z.theta)*cos(x1.theta) - sin(z.theta)*sin(x1.theta)
  gx_y = -fx_y; %cos(z.theta)*sin(x1.theta) + sin(z.theta)*cos(x1.theta)
  gx_theta = 0;

  gy_x = -fy_x;  %sin(z.theta)*cos(x1.theta) + cos(z.theta)*sin(x1.theta) 
  gy_y = -fy_y;  %sin(z.theta)*sin(x1.theta) - cos(z.theta)*cos(x1.theta) 
  gy_theta = 0;

  B = [ gx_x, gx_y, gx_theta;
  	gy_x, gy_y, gy_theta;
  	0, 0, 1];

  % Calculate error
  e_t = invt(v2t(z))*(invt(v2t(x1))*v2t(x2));
  e = t2v(e_t);

  
  

  

end;
