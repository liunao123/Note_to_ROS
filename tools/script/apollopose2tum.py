

# 不需要apollo环境，需要pip3 install cyber_record

# read_apollo_record.py
# from cyber_py3 import record
from cyber_record.record import Record
import os


def heading_to_quaternion(heading_deg):
    """
    将heading角度转换为四元数
    heading: 航向角（度），通常0度指北，顺时针为正
    返回四元数 [x, y, z, w] 格式
    """
    # 将度转换为弧度
    heading_rad = np.radians(heading_deg)
    
    # 只有yaw旋转，pitch=0, roll=0
    # 使用ZYX欧拉角顺序：[yaw, pitch, roll]
    r = R.from_euler('ZYX', [heading_rad, 0, 0])
    q = r.as_quat()  # [x, y, z, w]
    
    return q

def ned_to_enu_euler(yaw_ned, pitch_ned, roll_ned):
    """
    将NED坐标系下的欧拉角转换为ENU坐标系下的欧拉角
    NED: North-East-Down, ENU: East-North-Up
    """
    # NED到ENU的欧拉角转换关系:
    # ENU_yaw = NED_yaw + 90°  (从北向转到东向起算)
    # ENU_pitch = -NED_pitch   (俯仰角方向相反)
    # ENU_roll = -NED_roll     (横滚角方向相反)
    
    yaw_enu = -(yaw_ned + 90.0)
    # yaw_enu =  90.0 - yaw_ned - 180
    # 保持yaw在[-180, 180]范围内
    if yaw_enu > 180.0:
        yaw_enu -= 360.0
    elif yaw_enu < -180.0:
        yaw_enu += 360.0
        
    pitch_enu = -pitch_ned
    roll_enu = -roll_ned
    
    return yaw_enu, pitch_enu, roll_enu

def quaternion_to_matrix(q):
    """将四元数转换为4x4齐次变换矩阵"""
    x, y, z, w = q
    R_matrix = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
    return R_matrix

def pose_to_matrix(t, q):
    """将位置和四元数转换为4x4齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(q)
    T[:3, 3] = t
    return T

def matrix_to_pose(T):
    """将4x4齐次变换矩阵转换为位置和四元数"""
    t = T[:3, 3]
    R_matrix = T[:3, :3]
    r = R.from_matrix(R_matrix)
    q = r.as_quat()  # [x, y, z, w]
    return t, q

def process_apollo_records(input_directory, output_file):
    """
    Reads Apollo record files from a specified input directory,
    extracts /apollo/localization/pose and /apollo/sensor/gnss/odometry data, 
    and saves them as TUM format in separate files.
    """
    # if not os.path.exists(output_directory):
    #     os.makedirs(output_directory)

    # Define the output TUM file paths
    # single_output_tum_path = os.path.join(output_directory, "all_apollo_poses_0609_2.tum")
    single_output_tum_path = output_file
    gnss_output_tum_path = output_file.replace('.txt', '_gnss.txt')
    
    sorted_files = sorted( os.listdir(input_directory) )
    
    # 存储最新的heading值
    latest_heading = 0.0
    
    with open(single_output_tum_path, "w") as f_out, open(gnss_output_tum_path, "w") as f_gnss:
        total_msg_count = 0
        total_gnss_count = 0
        for item in sorted_files:
            file_path = os.path.join(input_directory, item)
            if os.path.isfile(file_path):
                msg_count_current_file = 0
                gnss_count_current_file = 0
                try:
                    # if 'record' in file_path:
                    #     pass
                    # else:
                    #     continue

                    print(file_path)

                    record = Record(file_path)
                    for channel_name, message, _ in record.read_messages():
                        # q_new = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]
                        if channel_name == "/apollo/localization/pose":
                            # 解析消息（需根据实际 proto 结构调整）
                            if hasattr(message, 'header') and hasattr(message, 'pose'):
                                header = message.header
                                pose = message.pose
                                timestamp = header.timestamp_sec
                                # timestamp = message.measurement_time
                                # print(timestamp)
 
                                tx2 = pose.position.x
                                ty2 = pose.position.y
                                tz2 = pose.position.z
                                qx2 = pose.orientation.qx
                                qy2 = pose.orientation.qy
                                qz2 = pose.orientation.qz
                                qw2 = pose.orientation.qw
                                f_out.write(f"{timestamp:.6f} {tx2:.6f} {ty2:.6f} {tz2:.6f} {qx2:.6f} {qy2:.6f} {qz2:.6f} {qw2:.6f}\n")
                                msg_count_current_file += 1
                                total_msg_count += 1
                            else:
                                print(f"Skipping message on channel {channel_name} in {item} due to missing header or pose attributes.")
                        
                        # if channel_name == "/apollo/sensor/gnss/odometry":
                        #     # 解析GNSS odometry消息 (apollo.localization.Gps)
                        #     if hasattr(message, 'header') and hasattr(message, 'localization'):
                        #         header = message.header
                        #         localization = message.localization
                        #         timestamp = header.timestamp_sec
                        #         tx = localization.position.x  
                        #         ty = localization.position.y  
                        #         tz = localization.position.z
                        #         qx = localization.orientation.qx
                        #         qy = localization.orientation.qy
                        #         qz = localization.orientation.qz
                        #         qw = localization.orientation.qw
                        #         f_gnss.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
                        #         gnss_count_current_file += 1
                        #         total_gnss_count += 1
                        #     else:
                        #         print(f"Skipping GNSS message on channel {channel_name} in {item} due to missing header or localization attributes.")
 
                    print(f"Extracted {msg_count_current_file} pose data points and {gnss_count_current_file} GNSS data points from {item}.")
                except Exception as e:
                    print(f"Error processing {file_path}: {e}")

    print(f"\nTotal extraction complete!")
    print(f"Pose data: {total_msg_count} points saved to {single_output_tum_path}")
    print(f"GNSS data: {total_gnss_count} points saved to {gnss_output_tum_path}")

if __name__ == "__main__":

    # Example usage:
    input_data_directory = "/mnt/nvme0n1p2/project/W2_pose/bag/"   
    output_results_directory = "/mnt/nvme0n1p2/project/W2_pose/bag/W7_poses_n.txt"  # Replace with your desired output directory
    process_apollo_records(input_data_directory, output_results_directory)

