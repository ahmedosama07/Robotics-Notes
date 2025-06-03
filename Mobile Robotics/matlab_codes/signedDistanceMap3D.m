%% Signed distance map helper
function sdf = signedDistanceMap3D(map3D)
    % Convert occupancyMap3D to 3D grid
    occ = occupancyMatrix(map3D);
    occ = double(occ >= 0.5);
    sdf = bwdist(~occ) - bwdist(occ);
    sdf = sdf * map3D.Resolution; % scale to metric units
end