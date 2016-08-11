close all; clear all;
[object_pose, tip_pose, wrench] = get_and_plot_data('pushing_data/rect1_json/motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=0.349.json', 'rect1', 1);
%[object_pose, tip_pose, wrench] = get_and_plot_data('pushing_data/rect1_json/motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=-1.047.json', 'rect1', 1);

N = 50;
[obj_pose, tip_pt, force, t_q] = interp_data(object_pose, tip_pose, wrench, N);
[wrench_local, twist_local] = compute_wrench_twist_local_frame(force, obj_pose, tip_pt, t_q);

pho = 0.05;
wrench_local(:,3) = wrench_local(:,3) / pho;
twist_local(:,3) = twist_local(:,3) * pho;

figure; plot3(wrench_local(:,1), wrench_local(:,2), wrench_local(:,3), 'r.');