function obsGrad = computeObstacleGradient(robot, traj, map3D, dt)
% Inputs:
% robot  : rigidBodyTree
% traj   : NxM matrix of joint angles
% map3D  : occupancyMap3D
% dt     : time step
% Output:
% obsGrad: NxM matrix of obstacle gradient

% Precompute SDF
sdf = signedDistanceMap3D(map3D);

N = size(traj,1);
numJoints = size(traj,2);
obsGrad = zeros(N, numJoints);

% Sampling points on robot (you can refine this!)
linkNames = {'link1','link2','link3','link4','link5','link6','link7'}; % customize for your robot

for i=1:N
    q_i = traj(i,:);
    grad_q = zeros(1,numJoints);

    for linkIdx = 1:numel(linkNames)
        % Get link transform
        tform = getTransform(robot, q_i, linkNames{linkIdx});
        p_link = tform(1:3,4)';

        % SDF gradient at this point
        [grad_sdf, dist] = getSDFGradient(sdf, p_link, map3D.Resolution);

        % Compute geometric Jacobian for link's CoM
        J = geometricJacobian(robot, q_i, linkNames{linkIdx});

        % Only linear part of Jacobian affects gradient (first 3 rows)
        Jv = J(1:3,:);

        % Chain rule to map workspace gradient to joint space
        grad_q = grad_q + (grad_sdf' * Jv);
    end

    obsGrad(i,:) = grad_q;
end

end