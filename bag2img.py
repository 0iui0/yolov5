import rosbag
from cv_bridge import CvBridge
import cv2
import os

def convert_rosbag_to_images(rosbag_file, image_topic, output_dir, image_skip):
    bag = rosbag.Bag(rosbag_file)
    bridge = CvBridge()
    count = 0

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if count % image_skip == 0:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = msg.header.stamp.to_nsec()
            image_filename = '{}.jpg'.format(timestamp)
            image_path = os.path.join(output_dir, image_filename)
            cv2.imwrite(image_path, cv_image)
        count += 1

    bag.close()
# Example usage
rosbag_file = 'rosbag.bag'
image_topic = '/camera/color/image_raw'
output_dir = 'images'
image_skip = 10
convert_rosbag_to_images(rosbag_file, image_topic, output_dir, image_skip)

