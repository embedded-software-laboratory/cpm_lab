import csv
import matplotlib
import matplotlib.pyplot as plt

#matplotlib.use("pgf")
#matplotlib.rcParams.update({
#    "pgf.texsystem": "pdflatex",
#    'font.family': 'serif',
#    'text.usetex': True,
#    'pgf.rcfonts': False,
#})

# specify id for which to evaluate
id = 1

# create variables to load the data into
timesteps = []
speed_profile = []
new_old_fallback = []
priorities = []
trajectory = []

# path
path = ""

# read file
with open(path + "evaluation_3.csv") as csvfile:
    eval_reader = csv.reader(csvfile, delimiter = ';', )
    for row in eval_reader:
        timesteps.append(list(map(int, row[0].strip(',').split(','))))
        speed_profile.append(list(map(float, row[1].strip(',').split(','))))
        new_old_fallback.append(list(map(int, row[3].strip(',').split(','))))
        priorities.append(list(map(int, row[4].strip(',').split(','))).index(id))
    

def average_timestep_speed(profile):
    average_speed_profile = []
    for segment in profile:
        sum = 0
        count = 0
        for spd in segment:
            sum += spd
            count +=1
        average_speed_profile.append(sum/count)
    return average_speed_profile

def average_speed(profile):
    sum_speed = 0
    count = 0
    for segment in profile:
        for spd in segment:
            sum_speed += spd
            count +=1
        
    return sum_speed / count


avg_speed = average_speed(speed_profile)
print(avg_speed)

fig, ax = plt.subplots()
ax.bar(['1'], [avg_speed])

plt.ylabel("speed")
plt.show()

#plt.savefig('average_speed.pgf')
