import rosbag
import numpy as np
import rospy
from bisect import bisect_left
import matplotlib.pyplot as plt

# === 配置区域 ===
bag_path = "/home/wilson/fast_livo_ws/src/direct_visual_lidar_calibration/bag/mid360/3.bag"

topics = {
"left": "/left_camera/image",
"lidar": "/livox/lidar/filter"
}

reference_topic = "lidar"
align_target = "left"
spike_threshold_ms = 50 # 异常跳变阈值

# === 函数定义 ===
# """从bag中提取topic对应的时间戳"""
def extract_timestamps(bag, topic_name):
    timestamps = []
    for _, msg, t in bag.read_messages(topics=[topic_name]):
        if (hasattr(msg, "header") and msg._has_header):
            timestamps.append(msg.header.stamp.to_sec())
        else:
            timestamps.append(t.to_sec())
    return np.array(timestamps)
    
    # """对target中的每个时间戳, 在ref中找最近时间戳, 返回差值数组和匹配点"""
def match_timestamps(ref_times, target_times):
    diffs = []
    matched_refs = []
    ref_sorted = np.sort(ref_times)
    for t in target_times:
            idx = bisect_left(ref_sorted, t)
    if idx == 0:
                ref = ref_sorted[0]
    elif idx == len(ref_sorted):
                ref = ref_sorted[-1]
    else:
        before = ref_sorted[idx - 1]
        after = ref_sorted[idx]
        ref = before if abs(t - before) < abs(t - after) else after
        matched_refs.append(ref)
        diffs.append(t - ref)
    return np.array(diffs), np.array(matched_refs)

# """移除突变点, 返回插入NaN的平滑序列 用于断线 """
def remove_spikes(diffs, threshold_ms=50):
    diffs_ms = diffs * 1e3
    clean_diffs = [diffs_ms[0]]
    for i in range(1, len(diffs_ms)):
        if (abs(diffs_ms[i] - diffs_ms[i - 1]) > threshold_ms):
            clean_diffs.append(np.nan)
        else:
            clean_diffs.append(diffs_ms[i])
    return np.array(clean_diffs)

# === 主程序 ===
if __name__ == "__main__":
    print(f"读取 rosbag: {bag_path}")
    bag = rosbag.Bag(bag_path)

    # 提取每个topic的时间戳, 并裁剪头尾3帧
    topic_times = {}
    for name, topic in topics.items():
        times = extract_timestamps(bag, topic)
    if len(times) > 6:
        times = times[3:-3]
        topic_times[name] = times
        print(f" {name} topic 共有 {len(times)} 帧时间戳")
        bag.close()
    
        # 对齐分析：left vs lidar
        diffs, matched_refs = match_timestamps(topic_times[reference_topic], topic_times[align_target])
        diffs_filtered = remove_spikes(diffs, threshold_ms=spike_threshold_ms)
        print(f"\n对齐分析 [{align_target}] → [{reference_topic}]")
        print(f"有效帧数: {len(diffs)}")
        print(f"平均时间差: {np.nanmean(diffs_filtered):.3f} ms")
        print(f"最大时间差: {np.nanmax(np.abs(diffs_filtered)):.3f} ms")
        print(f"标准差    : {np.nanstd(diffs_filtered):.3f} ms")
    
        # === 输出异常帧索引 ===
        print(f"\n异常时间差帧（跳变超过 ±{spike_threshold_ms} ms）:")
        for idx, diff in enumerate(diffs):
            if abs(diff * 1e3) > spike_threshold_ms:
                print(f"  - 帧 {idx:03d} | Δt = {diff*1e3:.2f} ms | image_time = {topic_times[align_target][idx]:.6f} | lidar_time = {matched_refs[idx]:.6f}")
    
        # === 绘图 ===
        plt.figure()
        plt.plot(diffs_filtered, label=f"{align_target} - {reference_topic}", linewidth=1.0)
        plt.title(f"Timestamp Alignment (Filtered): {align_target} vs {reference_topic}")
        plt.xlabel("Frame Index")
        plt.ylabel("Time Difference (ms)")
        plt.grid(True)
        plt.legend()
        plt.show()
