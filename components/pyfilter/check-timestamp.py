import json, sys

FILE = 'human_data_2p_descriptores.txt'
    
with open(FILE, 'r') as f:
    raw = f.read()
data = json.loads(raw)["data_set"]

r = [d['timestamp'] for d in data]
rs = set([x for x in r if r.count(x)>1])
print(len(r), len(rs))

