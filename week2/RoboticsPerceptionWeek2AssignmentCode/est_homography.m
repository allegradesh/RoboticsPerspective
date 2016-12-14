function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
A = zeros(8,9);
% x_logo = H * x_video, x_logo (x',y',1) x_video (x,y,1)
for i=1:size(logo_pts,1)
  % calculate axi
  % ax = zeros(1,9);
  % ax(1:2) = -video_pts(i,:);
  % ax(3) = -1;
  % ax(7) = video_pts(i,1)*logo_pts(i,1);
  % ax(8) = video_pts(i,2)*logo_pts(i,1);
  % ax(9) = logo_pts(i,1);
  ax = [ -video_pts(i,:),-1,0,0,0,logo_pts(i,1)*[video_pts(i,:),1]];
  % calculate ayi
  ay = zeros(1,9);
  ay(4:5) = -video_pts(i,:);
  ay(6) = -1;
  ay(7) = video_pts(i,1)*logo_pts(i,2);
  ay(8) = video_pts(i,2)*logo_pts(i,2);
  ay(9) = logo_pts(i,2);
  A(2*(i-1)+1,:) = ax;
  A(2*(i-1)+2,:) = ay;
end

[U, S, V] = svd(A);
% tmp is the last column of V (9*9 matrix), should be a 9*1 column vector
% (h11 h12 h13 h21 h22 h23 h31 h32 h33)'
tmp = V(:,size(V,2)); 

H = (reshape(tmp,3,3))';

end

