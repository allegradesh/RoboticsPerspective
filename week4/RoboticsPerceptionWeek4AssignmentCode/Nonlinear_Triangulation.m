function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix for both
%     cameras
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 
X = zeros(size(X0));
for i = 1:size(X0,1),
    X(i,:) = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2,...
        C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:)); 
end

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)

b = [x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
uvw1 = K*R1*[eye(3) -C1]*[X0 1]';
uvw2 = K*R2*[eye(3) -C2]*[X0 1]';
uvw3 = K*R3*[eye(3) -C3]*[X0 1]';
fx = [uvw1(1)/uvw1(3) uvw1(2)/uvw1(3) uvw2(1)/uvw2(3)...
    uvw2(2)/uvw2(3) uvw3(1)/uvw3(3) uvw3(2)/uvw3(3)]';

J1 = Jacobian_Triangulation(C1, R1, K, X0);
J2 = Jacobian_Triangulation(C2, R2, K, X0);
J3 = Jacobian_Triangulation(C3, R3, K, X0);
J = [J1' J2' J3']';

err = (b-fx)'*(b-fx);
deltaX = (J'*J)\J'*(b-fx);
X = X0 + deltaX';

%%%% how to find the minimum??
%uvw1n = K*R1*[eye(3) -C1]*[X 1]';
%uvw2n = K*R2*[eye(3) -C2]*[X 1]';
%uvw3n = K*R3*[eye(3) -C3]*[X 1]';
%fxn = [uvw1n(1)/uvw1n(3) uvw1n(2)/uvw1n(3) uvw2n(1)/uvw2n(3)...
%    uvw2n(2)/uvw2n(3) uvw3n(1)/uvw3n(3) uvw3n(2)/uvw3n(3)]';
%e = (b-fxn)'*(b-fxn);
%while e>1,
%    X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X);
%end
   
end

function J = Jacobian_Triangulation(C, R, K, X)
uiX = K(1,1)*R(1,:)+K(1,3)*R(3,:);
viX = K(1,1)*R(2,:)+R(3,:)*[K(2,3) K(2,3) K(1,3)]';
wiX = R(3,:);

uvw = K*R*[eye(3) -C]*[X 1]';
J = [(uvw(3)*uiX-uvw(1)*wiX)/(uvw(3)^2);...
    (uvw(3)*viX-uvw(2)*wiX)/(uvw(3)^2)];
end
