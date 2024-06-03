# path
reference_scan_path=/home/dxz/Desktop/data/zhuoer-global.pcd;
local_scan_path=/home/dxz/Desktop/data/zhuoer-local.pcd;
local_scan_pose_path=/home/dxz/Desktop/data/zhuoer-local-traj.txt;
output_path=/home/dxz/Desktop/data/;

# run
./build/demo ${reference_scan_path} ${local_scan_path} ${local_scan_pose_path} ${output_path}