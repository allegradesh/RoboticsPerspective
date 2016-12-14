function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 3) pose rotation

A = zeros(2*size(X,1),3*4);
for i = 1: size(X,1),
    tmp = K\[x(i,:) 1]';
    A(2*(i-1)+1,:) = [-X(i,:),-1,0,0,0,0,tmp(1)*X(i,1),tmp(1)*X(i,2),tmp(1)*X(i,3),tmp(1)];
    A(2*(i-1)+2,:) = [0,0,0,0,-X(i,:),-1,tmp(2)*X(i,1),tmp(2)*X(i,2),tmp(2)*X(i,3),tmp(2)];
    %A(2*(i-1)+1,:) = [-X(i,:),-1,0,0,0,0,x(i,1)*X(i,1),x(i,1)*X(i,2),x(i,1)*X(i,3),x(i,1)];
    %A(2*(i-1)+2,:) = [0,0,0,0,-X(i,:),-1,x(i,2)*X(i,1),x(i,2)*X(i,2),x(i,2)*X(i,3),x(i,2)];
end
[u,s,v] = svd(A);
Rt = (reshape(v(:,end),4,3))';

%I = eye(size(K));
%tmp = (I/K)*P;    or directly: tmp = K\P;
R = Rt(:,1:end-1);
t = Rt(:,end);
[U,D,V] = svd(R);
s = det(U*V'); % can be 1 or -1
R = s*U*V';
t = s*t/D(1,1);

C = -R'*t;
end


