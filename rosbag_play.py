#multi rosbag play with one command
import subprocess
import rosbag
import os
import shlex

def get_bag_times(bag_file):
    with rosbag.Bag(bag_file) as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
    return start_time, end_time

def play_bag_files(bag_files, directory, rate=1.0):
    previous_end_time = None
    for bag_file in bag_files:
        bag_path = os.path.join(directory, bag_file)
        start_time, end_time = get_bag_times(bag_path)
        
        # Escape the bag path to handle spaces
        escaped_bag_path = shlex.quote(bag_path)
        
        if previous_end_time is not None:
            time_diff = start_time - previous_end_time
            if time_diff > 0:
                # Skip time_diff seconds to align the start of this bag file
                subprocess.run(f"rosbag play {escaped_bag_path} --clock --start {time_diff} -r {rate}", shell=True)
            else:
                # No gap between bags, play immediately
                subprocess.run(f"rosbag play {escaped_bag_path} --clock -r {rate}", shell=True)
        else:
            # First bag file, just play it
            subprocess.run(f"rosbag play {escaped_bag_path} --clock -r {rate}", shell=True)

        previous_end_time = end_time

if __name__ == "__main__":
    prefix = "aqc_808_2024-08-20-03-2"  # 替换为你的前缀
    directory = "/media/yyp/My Book/week3_0820"  # ROS bag文件目录
    playback_rate = 5.0  # 设置倍速播放

    # List all bag files in the specified directory with the specified prefix, sorted by their names
    bag_files = sorted([f for f in os.listdir(directory) if f.endswith('.bag') and f.startswith(prefix)])

    if not bag_files:
        print(f"No bag files with prefix '{prefix}' found in the directory '{directory}'.")
    else:
        play_bag_files(bag_files, directory, rate=playback_rate)
