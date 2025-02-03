#!/usr/bin/env python

import os
import datetime
import rospy
from std_msgs.msg import String

# Buat direktori ~/data_logs jika belum ada
log_dir = os.path.expanduser('~/data_logs')
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# Nama file log tetap
log_file = os.path.join(log_dir, 'logger.txt')

def callback(data):
    # Dapatkan waktu saat ini
    now = datetime.datetime.now()
    day = now.strftime('%A')
    date = now.strftime('%Y-%m-%d')
    time = now.strftime('%H:%M:%S')

    # Format data yang akan disimpan
    log_entry = f"{day}, {date}, {time}, {data.data}\n"

    # Simpan data ke file log
    with open(log_file, 'a') as f:
        f.write(log_entry)

def listener():
    rospy.init_node('data_logger', anonymous=True)
    # rospy.Subscriber('udp_data', String, callback)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()