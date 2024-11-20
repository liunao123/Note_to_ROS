
# merge bag file

data_path="/opt/csg/slam/navs/v2_20241120"
echo ${data_path}
ls ${data_path}


python merge_bag.py  ${data_path}/0.bag  ${data_path}/00.bag ${data_path}/cinfo.bag
exit
python merge_bag.py  ${data_path}/1.bag  ${data_path}/11.bag ${data_path}/cinfo.bag
python merge_bag.py  ${data_path}/2.bag  ${data_path}/22.bag ${data_path}/cinfo.bag
python merge_bag.py  ${data_path}/3.bag  ${data_path}/33.bag ${data_path}/cinfo.bag
python merge_bag.py  ${data_path}/4.bag  ${data_path}/44.bag ${data_path}/cinfo.bag
# python merge_bag.py  ${data_path}/44.bag  ${data_path}/4.bag ${data_path}/camera_info.bag

# /home/liunao/Kalibr/v2_hall_extrisics//

# Convert Livox CustomMsg to PointCloud2
# python livox_to_pc2_for_dlvc.py




