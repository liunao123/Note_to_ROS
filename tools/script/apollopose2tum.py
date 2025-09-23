

# 不需要apollo环境，需要pip3 install cyber_record

# read_apollo_record.py
# from cyber_py3 import record
from cyber_record.record import Record
import os


def process_apollo_records(input_directory, output_file):
    """
    Reads Apollo record files from a specified input directory,
    extracts /apollo/localization/pose data, and saves it as TUM format
    in the given output directory.
    """
    # if not os.path.exists(output_directory):
    #     os.makedirs(output_directory)

    # Define the single output TUM file path
    # single_output_tum_path = os.path.join(output_directory, "all_apollo_poses_0609_2.tum")
    single_output_tum_path = output_file
    sorted_files = sorted( os.listdir(input_directory) )
    with open(single_output_tum_path, "w") as f_out:
        total_msg_count = 0
        for item in sorted_files:
            file_path = os.path.join(input_directory, item)
            if os.path.isfile(file_path):
                msg_count_current_file = 0
                try:
                    if 'record' in file_path:
                        pass
                    else:
                        continue

                    print(file_path)

                    record = Record(file_path)
                    for channel_name, message, _ in record.read_messages():                       
                        if channel_name == "/apollo/localization/pose":
                            # 解析消息（需根据实际 proto 结构调整）
                            if hasattr(message, 'header') and hasattr(message, 'pose'):
                                header = message.header
                                pose = message.pose
                                timestamp = header.timestamp_sec
                                tx = pose.position.x  
                                ty = pose.position.y  
                                tz = pose.position.z
                                qx = pose.orientation.qx
                                qy = pose.orientation.qy
                                qz = pose.orientation.qz
                                qw = pose.orientation.qw
                                f_out.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
                                msg_count_current_file += 1
                                total_msg_count += 1
                            else:
                                print(f"Skipping message on channel {channel_name} in {item} due to missing header or pose attributes.")
 
                    print(f"Extracted {msg_count_current_file} pose data points from {item}.")
                except Exception as e:
                    print(f"Error processing {file_path}: {e}")

    print(f"\nTotal extraction complete! All {total_msg_count} pose data points saved to {single_output_tum_path}")

if __name__ == "__main__":

    # Example usage:
    input_data_directory = "/home/tyjt/Desktop/drivewise_v101/data/park/record/msf2/"   
    output_results_directory = "/home/tyjt/Desktop/drivewise_v101/data/park/record/0702_msf_2.txt"  # Replace with your desired output directory
    process_apollo_records(input_data_directory, output_results_directory)

