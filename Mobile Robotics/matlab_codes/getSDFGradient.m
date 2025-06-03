%% Query SDF gradient at a point
function [grad, dist] = getSDFGradient(sdf, point, res)
    sz = size(sdf);
    idx = round(point / res) + floor(sz/2);

    % Clamp indices
    idx = max([1,1,1], min(sz, idx));

    % Central differences for gradient
    grad = zeros(1,3);
    for k=1:3
        idx_fw = idx; idx_fw(k)=min(sz(k), idx(k)+1);
        idx_bw = idx; idx_bw(k)=max(1, idx(k)-1);
        grad(k) = (sdf(idx_fw(1), idx_fw(2), idx_fw(3)) - sdf(idx_bw(1), idx_bw(2), idx_bw(3))) / (2*res);
    end

    dist = sdf(idx(1), idx(2), idx(3));
end