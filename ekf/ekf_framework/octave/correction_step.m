function [mu, sigma] = correction_step(mu, sigma, z, l)
    % Updates the belief, i. e., mu and sigma, according to the sensor model
    %
    % The employed sensor model is range-only.
    %
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    % z: structure containing the landmark observations, see
    %    read_data for the format
    % l: structure containing the landmark position and ids, see
    %    read_world for the format


    % Compute the expected range measurements.
    % This corresponds to the function h.
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    expected_ranges = zeros(size(z, 2), 1);
    H = [];
    for i = 1:size(z, 2)
        % Todo: Implement;
	landmarkId = z(i).id;
	dx = l(landmarkId).x - x;
	dy = l(landmarkId).y - y;
	qq = dx^2+dy^2;
	q = sqrt(qq);
	expected_ranges(i) = q;
	Hi = [-q*dx, -q*dy, 0];
	% Jacobian 
	H = [H;Hi];
    endfor

    % Measurements in vectorized form
    Z = zeros(size(z, 2), 1);
    for i = 1:size(z, 2)
        Z(i) = z(i).range; % Todo: Implement
    endfor

    R = diag(repmat([0.5], size(z, 2), 1));

    K = sigma*H'*inv(H*sigma*H'+R);
    mu = mu + K*(Z - expected_ranges);
    sigma = (eye(size(sigma,1))-K*H)*sigma;
end
