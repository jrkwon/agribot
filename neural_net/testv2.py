import pandas as pd
import os


#data_path = '/home/bimi/users/Elahe/Clean_intagribot/catkin_ws/e2e_data/mine4/2023-11-02-09-45-44'
#csv_path = '/home/bimi/users/Elahe/Clean_intagribot/catkin_ws/e2e_data/mine4/2023-11-02-09-45-44/2023-11-02-09-45-44.csv'
data_path = '/home/bimi/users/Elahe/agribot/catkin_ws/e2e_data/mine3/2023-10-31-14-56-16'
csv_path = '/home/bimi/users/Elahe/agribot/catkin_ws/e2e_data/mine3/2023-10-31-14-56-16/2023-10-31-14-56-16.csv'

df = pd.read_csv(csv_path)

new_csv = []
 
images = os.listdir(data_path)
image_name = []

for i in range(len(images)):
    if str(images[i])[-3:] == 'jpg':
          image_name.append(str(images[i]))

df2 = df.loc[df.image_fname.isin(image_name)]


    
          #print( i , df.loc[df.image_fname == image_name])

          #new_row = df.loc[df.image_fname == image_name]

          #df2.loc[len(df)] = df.loc[df.image_fname == image_name]


          #df2 = pd.concat([df2, df.loc[df.image_fname == image_name]])


         #df2.append(df.loc[df.image_fname == image_name])

print(image_name , len(image_name))

df2.to_csv("new_csv_n.csv")
