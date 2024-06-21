import cv2
import os

# Define parameters
input_dir = 'path/to/your/frames'  # Directory containing frames
output_file = 'output_video.avi'  # Output video file
fps = 30  # Frames per second

# Get list of all image files in the input directory
image_files = [f for f in os.listdir(input_dir) if os.path.isfile(os.path.join(input_dir, f))]
image_files.sort(key=lambda f: int(os.path.splitext(f)[0]))  # Sort files based on index

# Read the first image to get the width and height
first_image_path = os.path.join(input_dir, image_files[0])
frame = cv2.imread(first_image_path)
height, width, layers = frame.shape

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID') # TODO: Define good codec that does not add too much loss when later extracting the frames back from the video, maybe use MJPG?
video = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

# Read and write each frame to the video
for image_file in image_files:
    image_path = os.path.join(input_dir, image_file)
    frame = cv2.imread(image_path)
    video.write(frame)

# Release the VideoWriter object
video.release()

print(f"Video saved as {output_file}")
