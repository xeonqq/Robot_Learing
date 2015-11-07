% this function solves the odometry calibration problem
% given a measurement matrix Z.
% We assume that the information matrix is the identity
% for each of the measurements
% Every row of the matrix contains
% z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
% Z:	The measurement matrix
% X:	the calibration matrix
% returns the correction matrix X
function X = ls_calibrate_odometry(Z)
	% initial solution (the identity transformation)
	X = eye(3); 

	% TODO: initialize H and b of the linear system

	[row, cols] = size(Z);
	r = cols/2;
	Omega = eye(r);
	% TODO: loop through the measurements and update H and b
	% You may call the functions error_function and jacobian, see below
	% We assume that the information matrix is the identity.

	k=1; 
	% system (error function) is linear (also seen from jacobian, which does not contain state variable X)
	% so as the way least square method computes, which is e.T*Omega*e, so it will have only one global minimum (think of curve of x^2 or a parabola)
	% This means you will have only one iteration

	% But there is one condition, H has to be positive definite
	% otherwise, it will be a parabola on the top with single maximum
	while(k>0)
		  H = zeros(r^2, r^2);
		  b = zeros(1, r^2);
		  for i = 1:row
			  Ji = jacobian(i,Z);
			  H = H + Ji'*Omega*Ji;
			  e = error_function(i, X, Z);
			  b = b + e'*Omega*Ji;
		  end
		  % TODO: solve and update the solution
		  dx = -inv(H)*b';
		  X = X + reshape(dx,r,r)';
		  k = k-1;
	end
end

% this function computes the error of the i^th measurement in Z
% given the calibration parameters
% i:	the number of the measurement
% X:	the actual calibration parameters
% Z:	the measurement matrix, each row contains first the scan-match result
%       and then the motion reported by odometry
% e:	the error of the ith measurement
function e = error_function(i, X, Z)
	% TODO compute the error of each measurement
	e = Z(i,1:3)' - X*Z(i,4:end)'; 
end

% derivative of the error function for the ith measurement in Z
% i:	the measurement number
% Z:	the measurement matrix
% J:	the jacobian of the ith measurement
function J = jacobian(i, Z)
	% TODO compute the Jacobian
	[row, cols] = size(Z);
	r = cols/2;
	J = zeros(r, r^2);
	z =  Z(i,r+1:end);
	for j =1:r
		J(j,(j-1)*r+1:(j-1)*r+r) = -z;
	end

end
