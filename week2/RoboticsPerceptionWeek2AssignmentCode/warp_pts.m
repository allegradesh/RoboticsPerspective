function [ warped_pts ] = warp_pts( video_pts, logo_pts, sample_pts)
% warp_pts computes the homography that warps the points inside
% video_pts to those inside logo_pts. It then uses this
% homography to warp the points in sample_pts to points in the logo
% image
% Inputs:
%     video_pts: a 4x2 matrix of (x,y) coordinates of corners in the
%         video frame
%     logo_pts: a 4x2 matrix of (x,y) coordinates of corners in
%         the logo image
%     sample_pts: a nx2 matrix of (x,y) coordinates of points in the video
%         video that need to be warped to corresponding points in the
%         logo image
% Outputs:
%     warped_pts: a nx2 matrix of (x,y) coordinates of points obtained
%         after warping the sample_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% Complete est_homography first!
[ H ] = est_homography(video_pts, logo_pts);

% YOUR CODE HERE
n = size(sample_pts,1);
warped_pts = zeros(n,2);
for i=1:n
  % sample_tmp is a 3*1 vector
  sample_tmp = ones(3,1);
  sample_tmp(1:2) = (sample_pts(i,:))';
  
  scale = H(3,:)* sample_tmp;
  % warped_tmp is a 3*1 vector
  warped_tmp = H * sample_tmp / scale;
   
  warped_pts(i,:) = (warped_tmp(1:2))';
end

end

