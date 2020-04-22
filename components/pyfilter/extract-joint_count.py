import json, sys


if len(sys.argv) > 1:
    FILE = sys.argv[1]
else:
    FILE = 'human_data_1P_INF_R.txt'

MAX_PEOPLE = 5
joint_count = []
for i in range(MAX_PEOPLE):
    joint_count.append([0] * 18)

print("Loading file...")
with open(FILE, 'r') as f:
    raw = f.read()
data = json.loads(raw)

print("Processing...")
for line in data["data_set"]:
    for pos, p in enumerate(line["people"]):
        cont = len(p["joints"])
        joint_count[pos][cont] += 1


for p, person in enumerate(joint_count):
    if sum(person) > 0:
        print("Joints for person:", p+1)
        for p2, value in enumerate(person):
            print("\t", p2, ":", value)






