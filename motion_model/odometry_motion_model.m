%xt =[x,y,theta],  ut = [rot1,rot2,trans], alpha = [a1,a2,a3,a4]
function [px,py,ptheta] = odometry_motion_model(xt,ut,alpha)
	mu = 0;
	rot1 = ut(1); 
	rot2 = ut(2); 
	trans = ut(3);
	rot1_ = rot1 + normrnd(mu, alpha(1)*abs(rot1) + alpha(2)*trans);
	trans_ = trans + normrnd(mu, alpha(3)*trans + alpha(4)*(abs(rot1)+abs(rot2)));
	rot2_ = rot2 + normrnd(mu, alpha(1)*abs(rot2) + alpha(2)*trans);
	px = xt(1) + trans_*cos(xt(3) + rot1_) ;
	py = xt(2) + trans_*sin(xt(3) + rot1_);
	ptheta = xt(3) + rot1_ + rot2_;
endfunction

xt =[2.0,4.0,0.0];  ut = [pi/2,0,1]; alpha = [0.1,0.1,0.01,0.01];

xx = [];
yy = [];
tt = [];
for i = 1:1000
	[x,y,theta] = odometry_motion_model(xt,ut,alpha);
	xx = [xx, x];
	yy = [yy, y];
	tt = [tt, theta];
endfor

figure(1)
scatter(xx,yy)
title('2d banana shape with odometry motion model')
figure(2)
scatter3(xx,yy,tt)
title('3d banana shape with odometry motion model')
