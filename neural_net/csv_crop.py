import pandas as pd
import os


data_path = '/home/bimi/users/Elahe/agribot/catkin_ws/e2e_data/2023-10-31-14-56-16'
csv_path = '/home/bimi/users/Elahe/agribot/catkin_ws/e2e_data/mine3/2023-10-31-14-56-16/2023-10-31-14-56-16.csv'

df = pd.read_csv(csv_path)

df['image_fname'] = df['image_fname'].str[:-4]+'_crop.jpg'

df.to_csv('cropped_csv.csv')