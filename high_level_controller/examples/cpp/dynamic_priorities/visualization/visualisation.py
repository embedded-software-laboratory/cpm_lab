import csv
from typing import no_type_check
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
from numpy.core.shape_base import block

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

def read_speed(path, vehicles):
    # read files
    av_speed = []
    for n_veh in vehicles:
        # create variables to load the data into
        timesteps = []
        speed_profile = []
        new_old_fallback = []
        priorities = []
        trajectory = []
        for id in range(1,n_veh+1):
            with open(path + "/" + str(n_veh)+ "/evaluation_" + str(id) + ".csv") as csvfile:
                eval_reader = csv.reader(csvfile, delimiter = ';', )
                next(eval_reader, None) #skip header
                next(eval_reader, None)
                for row in eval_reader:
                    timesteps.append(list(map(int, row[0].strip(',').split(','))))
                    speed_profile.append(list(map(float, row[1].strip(',').split(','))))
                    new_old_fallback.append(list(map(int, row[3].strip(',').split(','))))
                    priorities.append(list(map(int, row[5].strip(',').split(','))).index(id))
        #print(speed_profile)
        av_speed.append(average_speed(speed_profile))
    return av_speed


def count_prios_used(new_old_fallback):
    new_prio_count = 0
    old_prio_count = 0
    fallback_prio_count = 0
    for prio in new_old_fallback:
        if prio == 0:
            new_prio_count +=1
        elif prio == 1:
            old_prio_count +=1
        elif prio == 2:
            fallback_prio_count +=1
    return [new_prio_count, old_prio_count, fallback_prio_count]

def read_prio_used(path, vehicles):
    # percent of total prios used
    percent_new_prio = []
    new_prio_count = []
    old_prio_count = []
    fallback_prio_count = []
    # for each mode add plot
    # read files
    for n_veh in vehicles:
        # create variables to load the data into
        new_old_fallback = []
        for id in range(1,n_veh+1):
            with open(path + "/" + str(n_veh)+ "/evaluation_" + str(id) + ".csv") as csvfile:
                eval_reader = csv.reader(csvfile, delimiter = ';', )
                next(eval_reader, None) #skip header
                next(eval_reader, None)
                for row in eval_reader:
                    new_old_fallback.append(list(map(int, row[3].strip(',').split(',')))[0])
        #print(new_old_fallback)

        npc, opc, fpc = count_prios_used(new_old_fallback)
        new_prio_count.append(npc/(npc+opc+fpc))
        old_prio_count.append(opc/(npc+opc+fpc))
        fallback_prio_count.append(fpc/(npc+opc+fpc))

    return [new_prio_count, old_prio_count, fallback_prio_count]
    

def plot_prio_used():
    fig, ax = plt.subplots(2,2)
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,19))

    hsteps = ["100", "200", "300", "400"]
    i = 0
    for x in ax.flat:
        print(hsteps[i])
        # fca
        new_prio_count, old_prio_count, fallback_prio_count = read_prio_used("./data/"+ hsteps[i]+"steps/2", vehicles)
        x.plot(vehicles, new_prio_count, "x", label="FCA")
        # random
        new_prio_count, old_prio_count, fallback_prio_count = read_prio_used("./data/"+ hsteps[i]+"steps/1", vehicles)
        x.plot(vehicles, new_prio_count, "+", label="Random")
        # static
        new_prio_count, old_prio_count, fallback_prio_count = read_prio_used("./data/"+ hsteps[i]+"steps/0", vehicles)
        x.plot(vehicles, old_prio_count, ".", label="Static")
        i +=1
     
    

    ax[0,0].set(title=r'$h=5s$')
    ax[0,1].set(title=r'$h=10s$')
    ax[1,0].set(title=r'$h=15s$')
    ax[1,1].set(title=r'$h=20s$')
    
    for x in ax.flat:
        x.set(xlabel=r'$n$ vehicles', ylabel=r'proposed prio. used $(\%)$')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for x in ax.flat:
        x.label_outer()
    # create one legend for the whole thingy    
    handles, labels = ax[1,1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left')

    # Figure Title

    plt.show()


def plot_avg_speed():
    fig, ax = plt.subplots(2,2)
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,19))

    hsteps = ["100", "200", "300", "400"]
    i = 0

    for x in ax.flat:
        print(hsteps[i])
        # fca
        av_speed = read_speed("./data/"+ hsteps[i]+"steps/2", vehicles)
        x.plot(vehicles, av_speed, "x", label="FCA")
        # random
        av_speed = read_speed("./data/"+ hsteps[i]+"steps/1", vehicles)
        x.plot(vehicles, av_speed, "+", label="Random")
        # static
        av_speed = read_speed("./data/"+ hsteps[i]+"steps/0", vehicles)
        x.plot(vehicles, av_speed, ".", label="Static")
        i+=1

    ax[0,0].set(title=r'$h=5s$')
    ax[0,1].set(title=r'$h=10s$')
    ax[1,0].set(title=r'$h=15s$')
    ax[1,1].set(title=r'$h=20s$')
    
    for x in ax.flat:
        x.set(xlabel=r'$n$ vehicles', ylabel=r'Speed $(m/s)$')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for x in ax.flat:
        x.label_outer()
    # create one legend for the whole thingy    
    handles, labels = ax[1,1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left')

    # Figure Title
    plt.show()

def plot_average_speed_zoomed():
    fig, ax = plt.subplots(2,2)
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,17))

    hsteps = ["100", "200", "300", "400"]
    i = 0

    for x in ax.flat:
        print(hsteps[i])
        # fca
        av_speed = read_speed("./data/"+ hsteps[i]+"steps/2", vehicles)
        x.plot(vehicles, av_speed, "x", label="FCA")
        # random
        av_speed = read_speed("./data/"+ hsteps[i]+"steps/1", vehicles)
        x.plot(vehicles, av_speed, "+", label="Random")
        # static
        av_speed = read_speed("./data/"+ hsteps[i]+"steps/0", vehicles)
        x.plot(vehicles, av_speed, ".", label="Static")
        i+=1

    ax[0,0].set(title=r'$h=5s$')
    ax[0,1].set(title=r'$h=10s$')
    ax[1,0].set(title=r'$h=15s$')
    ax[1,1].set(title=r'$h=20s$')
    
    for x in ax.flat:
        x.set_ylim([1.0, 1.4])
        x.set(xlabel=r'$n$ vehicles', ylabel=r'Speed $(m/s)$')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    for x in ax.flat:
        x.label_outer()
    # create one legend for the whole thingy    
    handles, labels = ax[1,1].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper left')

    # Figure Title
    plt.show()

def safe_stop(path, vehicles):
    stopped_after = []
    for n_veh in vehicles:
        # create variables to load the data into
        new_old_fallback = []
        speed_profile =[]
        for id in range(1,n_veh+1):
            tmp = []
            tmp_speed = []
            with open(path + "/" + str(n_veh)+ "/evaluation_" + str(id) + ".csv") as csvfile:
                eval_reader = csv.reader(csvfile, delimiter = ';', )
                next(eval_reader, None) #skip header
                next(eval_reader, None)
                for row in eval_reader:
                    tmp.append(list(map(int, row[3].strip(',').split(',')))[0])
                    tmp_speed.append(list(map(float, row[1].strip(',').split(','))))
            speed_profile.append(tmp_speed)
            new_old_fallback.append(tmp)
        #print(speed_profile)
        #print(new_old_fallback)
        i = 0
        stopped = False
        for case in new_old_fallback[0]:  
            if case == 2 and speed_profile[0][i] == [0,0,0,0,0,0,0,0]:
                stopped_after.append(i)
                stopped = True
                break
            i+=1
        if not stopped:
            stopped_after.append(-1)
    print(stopped_after)
    i = 0
    for stopped in stopped_after:
        #print(i)
        if stopped >=0:
            return i
        i +=1
    return i

def plot_stop():
    path = "./data/200steps/0"

    fig, ax = plt.subplots()
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,19))

    hsteps = ["100", "200", "300", "400"]
    i = 0
    data = []
    fca = []
    random = []
    static = []
    for steps in hsteps:
        print(steps)
        # fca
        f = safe_stop("./data/"+ steps+"steps/2", vehicles)
        r = safe_stop("./data/"+ steps+"steps/1", vehicles)
        s = safe_stop("./data/"+ steps+"steps/0", vehicles)
        fca.append(f)
        random.append(r)
        static.append(s)
    
    labels = ["FCA", "Random", "Static"]
    print(fca)
    #x_pos = [i for i, _ in enumerate(modes)]

    width = 0.15
    x = np.arange(len(hsteps))  # the label locations
    rects1 = ax.bar(x - width, fca, width, label='FCA')
    rects2 = ax.bar(x , random, width, label='Random')
    rects3 = ax.bar(x + width, static, width, label='Static')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    plt.ylabel(r'Successfull planning of $n$ vehicles over $\SI{180}{\second}$')
    
    plt.xticks(x, [r'$h=\SI{5}{\second}$', r'$h=\SI{10}{\second}$', r'$h=\SI{15}{\second}$', r'$h=\SI{20}{\second}$'])
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)
    ax.bar_label(rects3, padding=3)
    

    ax.legend()
    
    fig.tight_layout()
    plt.show()

# []
def consecutive_new_old_fallback(cases):
    plan_step_ms = 400
    proposed = []
    fallback = []
    stop = []
    not_proposed = []

    last_case = -1
    cons_occ = 0
    prop_inf = 0
    for case in cases:
        if case == 0:
            if last_case == case:
                cons_occ += 1
            else:
                not_proposed.append((prop_inf * plan_step_ms) / 1000)
                prop_inf = 0
                if last_case == 1:
                    fallback.append((cons_occ* plan_step_ms) / 1000)
                elif last_case == 2:
                    stop.append((cons_occ* plan_step_ms) / 1000)
                cons_occ = 1
                last_case = 0
        elif case == 1:
            prop_inf +=1
            if last_case == case:
                cons_occ += 1
            else:
                if last_case == 0:
                    proposed.append((cons_occ* plan_step_ms) / 1000)
                elif last_case == 2:
                    stop.append((cons_occ* plan_step_ms) / 1000)
                cons_occ = 1
                last_case = 1
        elif case == 2:
            prop_inf +=1
            if last_case == case:
                cons_occ += 1
            else:
                if last_case == 0:
                    proposed.append((cons_occ* plan_step_ms) / 1000)
                elif last_case == 1:
                    fallback.append((cons_occ* plan_step_ms) / 1000)
                cons_occ = 1
                last_case = 2
    if last_case == 0:
        proposed.append((cons_occ* plan_step_ms) / 1000)
        if prop_inf > 0:
            not_proposed.append((prop_inf* plan_step_ms) / 1000)
    elif last_case == 1:
        fallback.append((cons_occ* plan_step_ms) / 1000)
        not_proposed.append((prop_inf* plan_step_ms) / 1000)
    elif last_case == 2:
        not_proposed.append((prop_inf* plan_step_ms) / 1000)
        stop.append((cons_occ* plan_step_ms) / 1000)       
    return [proposed, fallback, stop, not_proposed]

# returns the interval duration of 
# [[proposed prios worked], [fallback worked], [stopping], [fallback or stopping]]
# in seconds
def boxplot_data(n_veh, horizon, mode):
    path = "./data/" + str(horizon) + "steps/" + str(mode)
   
    # create variables to load the data into
    new_old_fallback = []
    for id in range(1,n_veh+1):
        tmp = []
        with open(path + "/" + str(n_veh)+ "/evaluation_" + str(id) + ".csv") as csvfile:
            eval_reader = csv.reader(csvfile, delimiter = ';', )
            next(eval_reader, None) #skip header
            next(eval_reader, None)
            for row in eval_reader:
                tmp.append(list(map(int, row[3].strip(',').split(',')))[0])
        new_old_fallback.append(tmp)
    print(new_old_fallback[0])
    data = [[], [], [], []]
    for vehicle_data in new_old_fallback:
        eval = consecutive_new_old_fallback(vehicle_data)
        data[0] += eval[0]
        data[1] += eval[1]
        data[2] += eval[2]
        data[3] += eval[3]
        
    #data = consecutive_new_old_fallback(new_old_fallback[0])
    return data


def plot_feasibility_boxplot():
    case = 3
    n_veh = 16
    horizon = 200
    horizon_s = (horizon * 50)/1000 # h * minor_step / (ms/s)
    fca_data = boxplot_data(n_veh, horizon, 2)    
    random_data = boxplot_data(n_veh, horizon, 1)

    data = [fca_data[case], random_data[case]]
    print(data)

    fig1, ax1 = plt.subplots()
    labels = ["FCA", "Random"]
    ax1.set_title(r'$'+ str(n_veh) + r'$ vehicles, $h=\SI{' + str(horizon_s)+ r'}{\second}$')
    ax1.set_ylabel(r'interval duration $(\unit{\second})$')
    ax1.boxplot(data, labels=labels)

    plt.show()

def plot_speed_comparison_h():
    fig, ax = plt.subplots()
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,17))

    hsteps = ["100", "200", "300", "400"]
    i = 1

    # fca
    av_speed = read_speed("./data/"+ hsteps[i]+"steps/2", vehicles)
    ax.plot(vehicles, av_speed, "x", label="FCA")
    # random
    av_speed = read_speed("./data/"+ hsteps[i]+"steps/1", vehicles)
    ax.plot(vehicles, av_speed, "+", label="Random")
    # static
    av_speed = read_speed("./data/"+ hsteps[i]+"steps/0", vehicles)
    ax.plot(vehicles[0:11], av_speed[0:11], ".", label="Static")

    
    #ax.set_ylim([1.0, 1.4])
    ax.set(xlabel=r'$n$ vehicles', ylabel=r'average velocity $(\SI{}{\m\per\s})$')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    #ax.label_outer()
    # create one legend for the whole thingy    
    #handles, labels = ax.get_legend_handles_labels()
    ax.legend()
    fig.tight_layout()
    # Figure Title
    plt.show()


# reads and computes average plan timesstep durations of the vehicles
# returns [avg with 1 veh, ..., avg with n_veh]
def read_max_avg_plan_time(path, n_veh):
    avg_plan_time = []
    max_plan_time = []
    for n_veh in range(1, n_veh+1):
        # create variables to load the data into
        step_count = 0
        tmp_duration = 0
        tmp_max = 0
        for id in range(1,n_veh+1):
            with open(path + "/" + str(n_veh)+ "/evaluation_" + str(id) + ".csv") as csvfile:
                eval_reader = csv.reader(csvfile, delimiter = ';', )
                next(eval_reader, None) #skip header
                next(eval_reader, None)
                for row in eval_reader:
                    d = (list(map(float, row[7].strip(',').split(',')))[0])
                    tmp_duration += d
                    if d > tmp_max:
                        tmp_max = d
                    step_count += 1
        avg_plan_time.append(tmp_duration / step_count)
        max_plan_time.append(tmp_max)
    return [max_plan_time, avg_plan_time]

def plot_plan_time():
    n_veh = 16
    mode = 1
    horizon = 200
    horizon_s = (horizon * 50)/1000 # h * minor_step / (ms/s)

    path = "./data/" + str(horizon) + "steps/" + str(2)
    fca_data = read_max_avg_plan_time(path, n_veh)
    path = "./data/" + str(horizon) + "steps/" + str(1)
    random_data = read_max_avg_plan_time(path, n_veh)
    path = "./data/" + str(horizon) + "steps/" + str(0)
    static_data = read_max_avg_plan_time(path, 11)

    fig1, ax1 = plt.subplots()
    labels = ["FCA", "Random"]

    ax1.set_xlabel(r'$n$ vehicles')
    ax1.set_ylabel(r'Duration of plan step $(\unit{\ms})$')
    ax1.plot(np.arange(1, n_veh+1) ,fca_data[0], '-C0s', fillstyle='none', label = "max: FCA")
    ax1.plot(np.arange(1, n_veh+1) ,fca_data[1], '-C0d', fillstyle='none',label = "median: FCA")

    ax1.plot(np.arange(1, n_veh+1) ,random_data[0], '-C1s', fillstyle='none', label="max: Random")
    ax1.plot(np.arange(1, n_veh+1) ,random_data[1], '-C1d', fillstyle='none', label="median: Random")

    ax1.plot(np.arange(1, 11+1) ,static_data[0], '-C2s', fillstyle='none', label="max: Static")
    ax1.plot(np.arange(1, 11+1) ,static_data[1], '-C2d', fillstyle='none', label="median: Static")

    ax1.legend()
    

    plt.show()


def latex_export():
    matplotlib.use("pgf")
    matplotlib.rcParams.update({
        "pgf.texsystem": "pdflatex",
        'font.family': 'serif',
        'text.usetex': True,
        'pgf.rcfonts': False,
        "pgf.preamble": "\n".join([ # plots will use this preamble
        r"\usepackage[utf8]{inputenc}",
        r"\usepackage[T1]{fontenc}",
        r"\usepackage{siunitx}",
        ]),
    })

#latex_export()
#plot_avg_speed()
#plt.savefig('avg_speed.pgf')

#plot_prio_used()
#plt.savefig('new_prio_used.pgf')
#plot_average_speed_zoomed()
#plt.savefig('avg_speed_zoomed.pgf')

#plot_speed_comparison_h()
#plt.savefig('avg_speed_10s.pgf')

#plot_stop()
#plt.savefig('veh_until_infeasible.pgf')
plot_feasibility_boxplot()
#plt.savefig('consecutive_fail_boxplot.pgf')
#plot_plan_time()
#plt.savefig('plan_step_duration.pgf')
#plt.savefig("plan_step_duration.svg")
