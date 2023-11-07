import pandas as pd
import os

df = pd.read_csv('/home/bimi/users/Elahe/agribot/catkin_ws/e2e_data/mine3/2023-10-31-14-56-16/2023-10-31-14-56-16.csv')

image_path = '/home/bimi/users/Elahe/agribot/catkin_ws/e2e_data/mine3/2023-10-31-14-56-16/'
arr = os.listdir(image_path)
images_crop = []
for i in range(len(arr)):
    if arr[i][-8:] == "crop.jpg":
        images_crop.append(arr[i])
print(len(images_crop))

image_name = df['image_fname'].to_list()

print(len(image_name)) 

