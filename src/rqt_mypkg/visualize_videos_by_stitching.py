import os
import random
import cv2
import pandas as pd
from pathlib import Path

def create_combined_video(base_path, output_path, num_videos=10):
    for vid_index in range(num_videos):
        # Define the final video path for each run
        final_video_path = os.path.join(output_path, f"final_combined_video_{vid_index + 1}.avi")
        
        # Define video writer
        out = None

        # Iterate through each of the numbered folders
        for i in range(1, 18):
            folder_name = f"{i}_*"
            numbered_folders = list(Path(base_path).glob(folder_name))
            if not numbered_folders:
                continue
            selected_folder = random.choice(numbered_folders)
            date_folders = list(selected_folder.glob('*'))
            if not date_folders:
                continue
            selected_date_folder = random.choice(date_folders)
            
            csv_path = selected_date_folder / 'ee_csv.csv'
            if not csv_path.exists():
                continue
            df = pd.read_csv(csv_path)
            dataset_length = len(df)
            
            # Process images for each frame index
            for n in range(dataset_length):
                images = []
                widths = []
                for sub_folder, suffix in [('endo_psm2', '_psm2.jpg'), 
                                           ('left_img_dir', '_left.jpg'), 
                                           ('endo_psm1', '_psm1.jpg')]:
                    img_path = selected_date_folder / sub_folder / f"frame{str(n).zfill(6)}{suffix}"
                    if img_path.exists():
                        img = cv2.imread(str(img_path))
                        if img is not None:
                            if sub_folder == 'left_img_dir':
                                height = 480
                                width = int(img.shape[1] * (height / img.shape[0]))
                                img = cv2.resize(img, (width, height))
                            images.append(img)
                            widths.append(img.shape[1])

                if not images:
                    continue
                final_image = cv2.hconcat(images)
                
                # Calculate text position to center over the 'left_img_dir' image
                text_position_x = widths[0] + (widths[1] // 2) - 100  # Center text on the second image
                text = f"Folder: {selected_folder.stem}"
                cv2.putText(final_image, text, (text_position_x, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                if out is None:
                    h, w, _ = final_image.shape
                    out = cv2.VideoWriter(final_video_path, cv2.VideoWriter_fourcc(*'XVID'), 30, (w, h))
                out.write(final_image)

        # Release the video writer
        if out:
            out.release()
        print(f"Video {vid_index + 1} saved at {final_video_path}")

# Set the base and output path
base_path = "_recordings/tissue_4/"
output_path = "_recordings/tissue_4/combined_videos/"
create_combined_video(base_path, output_path)
