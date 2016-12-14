function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
R_ = [H(:,1) H(:,2) cross(H(:,1),H(:,2))];
[U,~,V] = svd(R_);
S_ = [1 1 det(U*V')];
R = U*diag(S_)*V';

t = H(:,3)/norm(H(:,1));
% YOUR CODE HERE: Project the points using the pose
proj_points_ = K*(R*render_points' + repmat(t,1,size(render_points,1)));
proj_points = (proj_points_(1:2,:)./repmat(proj_points_(3,:),2,1))';
end
