% Input: 
% pt_contacts: column vectors.
% pt_outward_normals: column vectors;
% mu: coefficient of friction at point of contacts, assume same for both contact points.
% pho: charateristic length. 
% Output: 
% fc_edges: 3*2*size(pc_contacts,2) matrix. Left edge is the left column.
% Normalized to 1(Directions). With torque component normalized by charateristic length pho. 
function [fc_edges] = ComputeFrictionConeEdges(pt_contacts, pt_outward_normals, mu, pho)
num_contacts = size(pt_contacts,2);
fc_edges = zeros(3,num_contacts * 2);
for ind_contact = 1:1:num_contacts
    axis_z = [0;0;1];
    axis_x = -[pt_outward_normals(:,ind_contact);0];
    axis_y = cross(axis_z, axis_x);
    F_x = 1;
    F_y = mu * F_x;
    F1 = F_x * axis_x + F_y * axis_y;
    F2 = F_x * axis_x - F_y * axis_y;
    pt_c = [pt_contacts(:, ind_contact); 0];
    tau1 = cross(pt_c, F1) / pho;
    tau2 = cross(pt_c, F2) / pho;
    W1 = F1 + tau1;
    W2 = F2 + tau2;
    W1 = W1 / norm(W1);
    W2 = W2 / norm(W2);
    fc_edges(:, ind_contact*2-1: ind_contact*2) = [W1,W2];
end
end

% Test cases:
% pt_contacts = [-0.025,0.025;-0.05,-0.05];
% pt_outward_normals = [0,0;-1,-1];
% pho = 0.05;
% mu = 0;
% [fc_edges] = ComputeFrictionConeEdges(pt_contacts, pt_outward_normals, mu, pho)

% fc_edges =
%
%          0         0         0         0
%     0.8944    0.8944    0.8944    0.8944
%    -0.4472   -0.4472    0.4472    0.4472

% If not normalized, then:
% fc_edges =
% 
%          0         0         0         0
%     1.0000    1.0000    1.0000    1.0000
%    -0.5000   -0.5000    0.5000    0.5000

% Change mu to very big;
% mu = 10000;
% [fc_edges] = ComputeFrictionConeEdges(pt_contacts, pt_outward_normals, mu, pho)
% 
% fc_edges =
% 
%    -0.7071    0.7071   -0.7071    0.7071
%     0.0001    0.0001    0.0001    0.0001
%    -0.7071    0.7071   -0.7071    0.7071