#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os
import signal
import sys
from datetime import datetime
from pynput import keyboard

log_data = []

def data_callback(msg):
    rospy.loginfo("Received data: %s", msg.data)
    log_data.append(msg.data)

def save_data(filename_suffix):
    directory = os.path.expanduser("~/data_logs")
    if not os.path.exists(directory):
        os.makedirs(directory)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_path = os.path.join(directory, f"data_log_{timestamp}_{filename_suffix}.txt")
    with open(log_file_path, 'w') as f:
        for line in log_data:
            f.write(line + "\n")
    rospy.loginfo("Data saved to %s", log_file_path)

def save_data_signal(signal, frame):
    save_data("exit")
    sys.exit(0)

def on_press(key):
    try:
        if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
            on_press.ctrl_pressed = True
        elif key == keyboard.KeyCode.from_char('s') and on_press.ctrl_pressed:
            save_data("runtime")
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        on_press.ctrl_pressed = False

on_press.ctrl_pressed = False

def main():
    rospy.init_node('data_logger', anonymous=True)
    rospy.Subscriber("udp_data", String, data_callback)

    signal.signal(signal.SIGINT, save_data_signal)
    signal.signal(signal.SIGTSTP, save_data_signal)

    # Set up keyboard listener for "Ctrl+S"
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rospy.spin()

if __name__ == '__main__':
    main()