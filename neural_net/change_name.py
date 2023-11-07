import pandas as pd
import os

data_path = '/home/bimi/users/Elahe/Clean_intagribot/Feeza dataset/2023-11-02-09-39-30'
csv_path = '/home/bimi/users/Elahe/Clean_intagribot/Feeza dataset/2023-11-02-09-39-30.csv'

df = pd.read_csv(csv_path)

df['image_fname'] = df['image_fname'].str[:-4] + 'n' + '.jpg'

print(df['image_fname'])
df.to_csv("new_csv_n1.csv")

'''
new_csv = []
 
images = os.listdir(data_path)
image_name = []
print(str(images))
print(str(images[1]))
print(str(images[1][:-4]))
print(str(images[1][:-4])+'n'+str(images[1][-4:]))

for i in range(len(images)):
    print(i)
    if str(images[i])[-3:] == 'jpg':
          image_name.append(str(images[i][:-4])+'n'+str(images[i][-4:]))
          #image_name.append(str(images[i]))
          #os.rename(data_path+'/'+str(images[i]),'/home/bimi/users/Elahe/Clean_intagribot/Feeza dataset/1'+'/'+str(images[i][:-4])+'n'+str(images[i][-4:]))

df2 = df.loc[df.image_fname.isin(image_name)]


    
          #print( i , df.loc[df.image_fname == image_name])

          #new_row = df.loc[df.image_fname == image_name]

          #df2.loc[len(df)] = df.loc[df.image_fname == image_name]


          #df2 = pd.concat([df2, df.loc[df.image_fname == image_name]])


         #df2.append(df.loc[df.image_fname == image_name])

print(image_name , len(image_name))

df2.to_csv("new_csv_n1.csv")

'''