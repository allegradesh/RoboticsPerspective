function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2
A = ones(size(x1,1),9);
A(:,1) = x1(:,1).*x2(:,1);
A(:,2) = x1(:,1).*x2(:,2);
A(:,3) = x1(:,1);
A(:,4) = x1(:,2).*x2(:,1);
A(:,5) = x1(:,2).*x2(:,2);
A(:,6) = x1(:,2);
A(:,7) = x2(:,1);
A(:,8) = x2(:,2);
[U,S,V] = svd(A);
x = V(:,9);
F = reshape(x,3,3);
[u,d,v] = svd(F);
d(3,3) = 0;
F = u*d*v';
end


