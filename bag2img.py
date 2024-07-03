import rosbag
from cv_bridge import CvBridge
import cv2
import os
from skimage.metrics import structural_similarity as ssim
import os
import subprocess
import cv2
import glob

def create_video_from_images(image_dir, output_video_path, fps=30):
    # List all .jpg files in the directory
    image_files = sorted(glob.glob(os.path.join(image_dir, '*.jpg')))
    
    # Check if there are any images
    if not image_files:
        raise ValueError("No .jpg images found in the directory.")
    
    # Read the first image to get dimensions
    image_path = image_files[0]
    image = cv2.imread(image_path)
    height, width, _ = image.shape
    
    # Create a file list for ffmpeg
    file_list_path = os.path.join(image_dir, 'file_list.txt')
    with open(file_list_path, 'w') as f:
        for image_file in image_files:
            f.write(f"file '{os.path.relpath(image_file, image_dir)}'\n")
    
    # ffmpeg command with file list input
    command = [
        'ffmpeg',
        '-y',
        '-f', 'concat',
        '-safe', '0',
        '-i', file_list_path,
        '-c:v', 'libx264',
        '-profile:v', 'high',
        '-crf', '20',
        '-pix_fmt', 'yuv420p',
        '-r', str(fps),
        image_dir + output_video_path
    ]
    
    # Run the ffmpeg command
    subprocess.run(command)
def convert_rosbag_to_images(rosbag_file, image_topic, output_dir, image_skip, ssim_threshold):
    bag = rosbag.Bag(rosbag_file)
    bridge = CvBridge()
    count = 0
    prev_image = None

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if count % image_skip == 0:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = msg.header.stamp.to_nsec()
            image_filename = f'{timestamp}.jpg'
            image_path = os.path.join(output_dir, image_filename)

            # Check if the image is too similar to the previous one
            if prev_image is not None:
                gray_prev = cv2.cvtColor(prev_image, cv2.COLOR_BGR2GRAY)
                gray_current = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                similarity = ssim(gray_prev, gray_current)
                if similarity > ssim_threshold:
                    continue

            cv2.imwrite(image_path, cv_image)
            prev_image = cv_image
        count += 1

    bag.close()
# List of rosbag files
rosbag_files = [
]

# Common parameters
image_topic = ''
output_dir = ''
image_skip = 2
ssim_threshold = 0.95  # Set the threshold for similarity
video_filename = "test.mp4"
# Process each rosbag file
for rosbag_file in rosbag_files:
    convert_rosbag_to_images(rosbag_file, image_topic, output_dir, image_skip, ssim_threshold)
    create_video_from_images(output_dir, video_filename)
    
