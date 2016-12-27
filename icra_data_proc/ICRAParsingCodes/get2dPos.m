% Input: Cartesian Pose (x y z qw qx qy qz) of row matrix.
%        Transformation of local object frame w.r.t mocap body frame.
function [ pos_2d ] = get2dPos(cart_pos, H_tf, unit)
if (nargin == 2)
    unit = 1000;
end
trans = cart_pos(:, 1:3);
quat = cart_pos(:,4:end);

N = size(trans, 1);
pos_2d = zeros(N,3);

for i = 1:1:N
    H_m = getHomogTransf(trans(i,:)', quat(i,:)');
    % Apply transform to get local object frame pose. 
    H_obj = H_m * H_tf;
    
    R = H_obj(1:3, 1:3);
    t = H_obj(1:3, 4);
    q = qGetQ(R);
    % x and y.
    pos_2d(i, 1:2) = t(1:2) / unit;
    % theta.
    % pos_2d(i, 3) = atan2(q(4), q(1)) * 2 * 180 / pi;
    pos_2d(i, 3) = atan2(q(4), q(1)) * 2;
    pos_2d(i,3) = mod(pos_2d(i,3), 2*pi);
end

end

