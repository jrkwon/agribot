import pandas as pd

df1 = pd.read_csv('/home/bimi/users/Elahe/Clean_intagribot/2023-11-02-09-39-30.csv')
df2 = pd.read_csv('/home/bimi/users/Elahe/Clean_intagribot/2023-11-02-09-45-44.csv')
print(df2.shape)
print(df1.shape)
merged_df = pd.merge(df1, df2,how='outer')
merged_df.to_csv("merged.csv")