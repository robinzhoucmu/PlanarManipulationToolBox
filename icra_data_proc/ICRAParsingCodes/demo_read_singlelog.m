file_name = 'SensorLogs/10_130_10_10_10_130/exp_08_17_50.txt';
trans = [50;50;0];
H_tf = [eye(3,3), trans;
      0,0,0,1];
R_tool = [sqrt(2)/2, sqrt(2)/2;
          sqrt(2)/2, -sqrt(2)/2]';


% Parameters for trianglular block.         
Tri_mass = 1.518;
% Black board.
mu_f_blk = 4.2 / (Tri_mass * 9.8);
% Wood board.
mu_f_wood = 5.0 / (Tri_mass * 9.8);
Tri_com = [0.15/3; 0.15/3];
Tri_pho = 0.05;
unit_scale = 1000;
[ record_log ] = ExtractFromLog(file_name, Tri_pho, R_tool, H_tf, unit_scale);

% Construct triangular push object.
