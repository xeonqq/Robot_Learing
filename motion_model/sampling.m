function p = gaussian(x, u, theta)
	p = 1/(theta*sqrt(2*pi))*exp(-1/2*((x-u)/theta)^2);
endfunction


%p = gaussian(1.2,0,0.4)

function sample = unifrnd_sampling(u,theta)
	sample = sum(unifrnd(-theta,theta,[12,1]))/2;
endfunction

function x = rejection_sampling(u,theta)
	b = 4*theta;
	do
		x = unifrnd(-b,b);
		c = unifrnd(0,1/(theta*sqrt(2*pi)));
	until (gaussian(x,u,theta) > c)
endfunction

function x = box_muller_sampling(u,theta)
	u1 = unifrnd(0,1);
	u2 = unifrnd(0,1);
	x = cos(2*pi*u1)*sqrt(-2*log(u2));
endfunction

u = 0; theta = 0.4;
iterations = 1000

tic()
for i = 1:iterations
	s1 = unifrnd_sampling(u,theta);
endfor
unifrnd_sampling_time =toc()

tic()
for i = 1:iterations
	s2 = rejection_sampling(u,theta);
endfor
rejection_sampling_time=toc()

tic()
for i = 1:iterations
	s3 = box_muller_sampling(u,theta);
endfor
box_muller_sampling_time =toc()

tic()
for i = 1:iterations
	s4 = normrnd(u,theta);
endfor
normrnd_time=toc()
