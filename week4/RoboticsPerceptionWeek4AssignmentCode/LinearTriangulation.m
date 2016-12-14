function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     K  - size (3 x 3) camera intrsinc parameter for both cameras
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points
P1 = K*R1*[eye(3) -C1];
P2 = K*R2*[eye(3) -C2];
x_1 = [x1 ones(size(x1,1),1)];
x_2 = [x2 ones(size(x2,1),1)];
X = ones(size(x1,1),3);
for i = 1:size(x1,1),
  skew1= Vec2Skew(x_1(i,:));
  skew2= Vec2Skew(x_2(i,:));
  A = [skew1*P1;skew2*P2];
  [u,d,v] = svd(A);
  X_ = v(:,end)/v(end,end);
  X(i,:) = (X_(1:end-1))';
end
end