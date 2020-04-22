import json, sys

FILE = 'human_data_textured2.txt'
FILEW = 'human_data_textured2_large.txt'
    
print("Working...")
with open(FILE, 'r') as f:
    raw = f.read()
data = json.loads(raw)
total = len(data['data_set'])
del data['data_set'][1200:-1]

with open(FILEW, 'w') as f:
    json.dump(data, f)

print("Wrote ", len(data['data_set']), " elements of a todal of ", total)

