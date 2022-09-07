import csv
from typing import no_type_check
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator
import matplotlib.font_manager
from numpy.core.shape_base import block
from scipy.signal import hilbert, chirp
from scipy.interpolate import interp1d

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

    hsteps = "200" #["100", "200", "300", "400"]
    seed = 0
    i = 0

    for x in ax.flat:
        print(hsteps[i])
        # fca
        av_speed = read_speed("./data/"+ hsteps+"/2", vehicles)
        x.plot(vehicles, av_speed, "x", label="FCA")
        # random
        av_speed = read_speed("./data/"+ hsteps[i]+"/1", vehicles)
        x.plot(vehicles, av_speed, "+", label="Random")
        # static
        av_speed = read_speed("./data/"+ hsteps[i]+"/0", vehicles)
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
    fig, ax = plt.subplots(1)
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,20))

    hsteps = "200"#, "200", "300", "400"]
    colors = ['b', 'g', 'r', 'c']
    seeds = ["0", "1", "2", "3"]
    symbols = ['x', '+', '.', '|']

    i=0
    for seed in seeds:
        # fca
        av_speed = read_speed("./data_seed_" + seed  + "/" + hsteps+"/2", vehicles)
        ax.plot(vehicles, av_speed, "x", label="FCA", color=colors[i])

        #f = interp1d(vehicles, av_speed)
        #x_new = np.linspace(1, 19, num=1000, endpoint=True)
        #analytic_signal = hilbert(f(x_new))
        #amplitude_envelope = np.abs(analytic_signal)
        #ax.plot(x_new, amplitude_envelope, label="envelope")
        
        # random
        av_speed = read_speed("./data_seed_" + seed  + "/" + hsteps+"/1", vehicles)
        ax.plot(vehicles, av_speed, "+", label="Random", color=colors[i])
        # static
        av_speed = read_speed("./data_seed_" + seed  + "/" + hsteps+"/0", vehicles)
        ax.plot(vehicles, av_speed, ".", label="Static", color=colors[i])
        i+=1

    
    
    #ax[0,0].set(title=r'$h=5s$')
    #ax[0,1].set(title=r'$h=10s$')
    #ax[1,0].set(title=r'$h=15s$')
    #ax[1,1].set(title=r'$h=20s$')
    
    #for x in ax.flat:
        #x.set_ylim([1.0, 1.4])
    ax.set(xlabel=r'$n$ vehicles', ylabel=r'Speed $(m/s)$')

    # Hide x labels and tick labels for top plots and y ticks for right plots.
    #for x in ax.flat:
    ax.label_outer()
    # create one legend for the whole thingy    
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels)

    # Figure Title
    plt.show()

def safe_stop(path, vehicles, horizon_t):
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
        #  May lead to wrong detections, v1 may stop coincidentally when no feasible solution. Ideas:
        #  `case == 2` consecutively for t_h/dt steps
        i = 0
        stopped = False
        consecutively_stop = 0

        for case in new_old_fallback[0]:  
            if case == 2: 
                all_stopped = True
                for id in range(0, n_veh):
                    all_stopped = all_stopped and (speed_profile[id][i] == [0,0,0,0,0,0,0,0])
                if all_stopped:
                    stopped_after.append(i)
                    stopped = True
                    break
            i+=1
        if not stopped:
            stopped_after.append(-1)
    print(stopped_after)
    i = 0
    first_stop = 0
    for stopped in stopped_after:
        #print(i)
        if stopped >=0:
            if first_stop == 0:
                first_stop = i
        if first_stop != 0 and stopped < 0:
            print("######################################################################")
            print("Infeasible with " + str(first_stop) + "vehicles but feasible again with " + str(i) + "vehicles.")
            print("######################################################################")
            
        i +=1
    return first_stop



# Plots at how many vehicles the respective mode becomes infeasible
def plot_stop():
    path = "./data/200steps/0"

    fig, ax = plt.subplots()
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,19))

    steps = "200" # ["100", "200", "300", "400"]
    seeds = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "100", "101", "102", "103", "104", "105", "106", "107", "108", "109", "110", "111", "112", "113", "114", "115", "116", "117", "119", "120"]#, "2", "3", "4"]
    i = 0
    data = []
    fca = []
    random = []
    static = []
    for seed in seeds:
        print(steps)
        # fca
        f = safe_stop("./data_seed_" + seed  + "/"+ steps + "/2", vehicles, int(steps, 10))
        r = safe_stop("./data_seed_" + seed  + "/"+ steps + "/1", vehicles, int(steps, 10))
        s = safe_stop("./data_seed_" + seed  + "/"+ steps + "/0", vehicles, int(steps, 10))
        fca.append(f)
        random.append(r)
        static.append(s)
    
    labels = ["FCA", "Random", "Static"]
    print(fca)
    #x_pos = [i for i, _ in enumerate(modes)]

    width = 0.15
    x = np.arange(len(seeds))  # the label locations
    rects1 = ax.bar(x - width, fca, width, label='FCA')
    rects2 = ax.bar(x , random, width, label='Random')
    rects3 = ax.bar(x + width, static, width, label='Static')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    plt.ylabel(r'Successfull planning of $n$ vehicles over $\SI{180}{\second}$')
    
    plt.xticks(x, seeds)
    #plt.xticks(x, [r'$h=\SI{10}{\second}$'])
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)
    ax.bar_label(rects3, padding=3)
    

    ax.legend()
    
    fig.tight_layout()
    plt.show()

def plot_stop_boxplot():
    
    plt.rcParams["figure.figsize"] = (3.5,1.8)
    fig, ax = plt.subplots()
    
    # set height
    #fig.set_figheight(3)
    
    vehicles = list(range(1,19))

    steps = "200" # ["100", "200", "300", "400"]
    seeds = ["0", "1", "2", "3", "4", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "100", "101", "102", "103", "104", "105", "106", "107", "108", "109", "110", "111", "112", "113", "114", "115", "116", "117", "119", "120",]#, "2", "3", "4"]
    #seeds = ["100", "101", "102", "103", "104", "105", "106", "107", "108", "109", "110", "111", "112", "113", "114", "115", "116", "117", "118", "119", "120",]#, "2", "3", "4"]
    i = 0
    data = []
    fca = []
    random = []
    static = []
    for seed in seeds:
        print(seed)
        # fca
        f = safe_stop("./data_seed_" + seed  + "/"+ steps + "/2", vehicles, int(steps, 10))
        r = safe_stop("./data_seed_" + seed  + "/"+ steps + "/1", vehicles, int(steps, 10))
        s = safe_stop("./data_seed_" + seed  + "/"+ steps + "/0", vehicles, int(steps, 10))
        fca.append(f)
        random.append(r)
        static.append(s)
    
    labels = [r'$p_{\text{FCA}}$', r'$p_{r}$', r'$p_{s}$']
    
    plt.yticks([0,0.3,0.6])
    #plt.yticks(np.arange(min(x), max(x)+1, 1.0))
    #plt.tick_params(axis='y', pad=0.1) 
    ax.boxplot([fca, random, static], positions=[0,0.3,0.6], widths=[0.2,0.2,0.2], labels=labels, vert=False)
    #ax.set_title(r'Number of vehicles feasible for $\SI{180}{\second}$')
    ax.set_xlabel(r'Number of vehicles $N_A$')
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    
    plt.ylim([-0.2, 0.8])
    plt.tight_layout()



# Plots at how many vehicles the respective mode becomes infeasible
def plot_stop_mult_h():
    path = "./data/200steps/0"

    fig, ax = plt.subplots()
    
    #ax.set_xlim([0,19])
    #ax1.xaxis.set_ticks(np.arange(0, 19, 1))
    vehicles = list(range(1,19))

    hsteps = ["200"] # ["100", "200", "300", "400"]
    i = 0
    data = []
    fca = []
    random = []
    static = []
    for steps in hsteps:
        print(steps)
        # fca
        f = safe_stop("./data/"+ steps+"steps/2", vehicles, int(steps, 10))
        r = safe_stop("./data/"+ steps+"steps/1", vehicles, int(steps, 10))
        s = safe_stop("./data/"+ steps+"steps/0", vehicles, int(steps, 10))
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
    
    #plt.xticks(x, [r'$h=\SI{5}{\second}$', r'$h=\SI{10}{\second}$', r'$h=\SI{15}{\second}$', r'$h=\SI{20}{\second}$'])
    plt.xticks(x, [r'$t_H=\SI{10}{\second}$'])
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)
    ax.bar_label(rects3, padding=3)
    

    ax.legend()
    
    fig.tight_layout()
    plt.show()

def interval_unit(steps, plan_step_ms, unit):
    if unit == "steps":
        return steps
    else:
        (steps * plan_step_ms) / 1000

# []
def consecutive_new_old_fallback(cases):
    unit = "steps" # other option seconds

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
                not_proposed.append(interval_unit(prop_inf, plan_step_ms, unit))
                prop_inf = 0
                if last_case == 1:
                    fallback.append(interval_unit(cons_occ, plan_step_ms, unit))
                elif last_case == 2:
                    stop.append(interval_unit(cons_occ, plan_step_ms, unit))
                cons_occ = 1
                last_case = 0
        elif case == 1:
            prop_inf +=1
            if last_case == case:
                cons_occ += 1
            else:
                if last_case == 0:
                    proposed.append(interval_unit(cons_occ, plan_step_ms, unit))
                elif last_case == 2:
                    stop.append(interval_unit(cons_occ, plan_step_ms, unit))
                cons_occ = 1
                last_case = 1
        elif case == 2:
            prop_inf +=1
            if last_case == case:
                cons_occ += 1
            else:
                if last_case == 0:
                    proposed.append(interval_unit(cons_occ, plan_step_ms, unit))
                elif last_case == 1:
                    fallback.append(interval_unit(cons_occ, plan_step_ms, unit))
                cons_occ = 1
                last_case = 2
    if last_case == 0:
        proposed.append(interval_unit(cons_occ, plan_step_ms, unit))
        if prop_inf > 0:
            not_proposed.append(interval_unit(prop_inf, plan_step_ms, unit))
    elif last_case == 1:
        fallback.append(interval_unit(cons_occ, plan_step_ms, unit))
        not_proposed.append(interval_unit(prop_inf, plan_step_ms, unit))
    elif last_case == 2:
        not_proposed.append(interval_unit(prop_inf, plan_step_ms, unit))
        stop.append(interval_unit(cons_occ, plan_step_ms, unit))       
    return [proposed, fallback, stop, not_proposed]

# returns the interval duration of 
# [[proposed prios worked], [fallback worked], [stopping], [fallback or stopping]]
# in seconds
def boxplot_data(n_veh, horizon, mode, seed):
    path = "./data_seed_"+ seed + "/" + str(horizon) + "/" + str(mode) +"/"
   
    # create variables to load the data into
    new_old_fallback = []
    for id in range(1,n_veh+1):
        tmp = []
        with open(path + str(n_veh)+ "/evaluation_" + str(id) + ".csv") as csvfile:
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

def cases_percentage(fca_data, random_data):
    count_f = 0
    count_inf = 0
    count_stop = 0

    fca_feasible = sum(fca_data[0])
    fca_fallback = sum(fca_data[1])
    fca_infeasible = sum(fca_data[3])
    fca_stop = sum(fca_data[2])


    random_feasible = sum(random_data[0])
    random_fallback = sum(random_data[1])
    random_infeasible = sum(random_data[3])
    random_stop = sum(random_data[2])

    total_steps = fca_feasible + fca_fallback + fca_stop
    print("total steps: " + str(total_steps))
    print("FCA #####################")
    print("percent feasible: " + str(fca_feasible/total_steps))
    print("percent infeasible: " + str(fca_infeasible/total_steps))
    print("percent stopping: " + str(fca_stop/total_steps))
    print("percent fallback worked: " + str(fca_fallback/total_steps))
    print("Random #####################")
    print("percent feasible: " + str(random_feasible/total_steps))
    print("percent infeasible: " + str(random_infeasible/total_steps))
    print("percent stopping: " + str(random_stop/total_steps))
    print("percent fallback worked: " + str(random_fallback/total_steps))


def plot_case_boxplot():
    

    plt.rcParams["figure.figsize"] = (3.5,2.25)
    seed = "4"
    n_veh = 17
    horizon = 200
    horizon_s = (horizon * 50)/1000 # h * minor_step / (ms/s)
    fca_data = boxplot_data(n_veh, horizon, 2, seed)    
    random_data = boxplot_data(n_veh, horizon, 1, seed)

    case_f = 0 # proposed prios worked
    data_feasible = [fca_data[case_f], random_data[case_f]]

    case_inf = 3 # fallback or safe stop is used
    data_infeasible = [fca_data[case_inf], random_data[case_inf]]

    case_stop = 2
    data_stop = [fca_data[case_stop], random_data[case_stop]]

    #fig1, ax1 = plt.subplots()
    # continuous proposed feasible and continuos infeasible 2 subplots:
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
    #fig.suptitle(r'$'+ str(n_veh) + r'$ vehicles, $t_H=\SI{' + str(horizon_s)+ r'}{\second}$')

    labels = [r'$p_{\text{FCA}}$', r'$p_{r}$']
    #ax1.set_title(r'Feasible Intervals')
    ax1.set_ylabel(r'Consecutive Timesteps')
    ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax1.boxplot(data_feasible, labels=labels)
    
    #ax2.set_title(r'Infeasible Intervals')
    ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
    #ax2.set_ylabel(r'interval duration $(\unit{\second})$')
    ax2.boxplot(data_infeasible, labels=labels)

    #ax3.set_title(r'Stopping Intervals')
    ax3.yaxis.set_major_locator(MaxNLocator(integer=True))
    #ax3.set_ylabel(r'interval duration $(\unit{\second})$')
    ax3.boxplot(data_stop, labels=labels)

    
    # save two subplots:
    # Save just the portion _inside_ the second axis's boundaries
    # plus pad the saved area by 20% in the x-direction and 15% in the y-direction
    # extent = ax1.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    # fig.savefig('feasible_boxplot.pgf', bbox_inches=extent.expanded(1.2, 1.15))
    # extent = ax2.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    # fig.savefig('infeasible_boxplot.pgf', bbox_inches=extent.expanded(1.2, 1.15))
    # extent = ax3.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    # fig.savefig('stopping_boxplot.pgf', bbox_inches=extent.expanded(1.2, 1.15))

    plt.show()
    cases_percentage(fca_data, random_data)
    #combined
    fig.savefig('feasib_infeasib_boxplot_combined.pgf')

    # ------------------------------------------------------ sperate plots
    size = (1.29,2.25)
    size2 = (1.1, 2.25)
    plt.cla()
    plt.rcParams["figure.figsize"] = size
    fig, ax = plt.subplots()
    ax.set_ylabel(r'Consecutive Timesteps')
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.boxplot(data_feasible, widths=[0.4,0.4], labels=labels)
    fig.tight_layout()
    fig.savefig('feasible_boxplot.pgf')

    plt.cla()
    plt.rcParams["figure.figsize"] = size2
    fig, ax = plt.subplots()
    #ax.set_ylabel(r'Consecutive Timesteps')
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.boxplot(data_infeasible, widths=[0.4,0.4], labels=labels)
    fig.tight_layout()
    fig.savefig('infeasible_boxplot.pgf')

    plt.cla()
    plt.rcParams["figure.figsize"] = size2
    fig, ax = plt.subplots()
    #ax.set_ylabel(r'Consecutive Timesteps')
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.boxplot(data_stop, widths=[0.4,0.4], labels=labels)
    fig.tight_layout()
    fig.savefig('stopping_boxplot.pgf')



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
    plt.rcParams["figure.figsize"] = (3.5, 3.5)
    seed ="4"
    n_veh = 17
    mode = 1
    horizon = 200
    horizon_s = (horizon * 50)/1000 # h * minor_step / (ms/s)

    path = "./data_seed_" + seed +"/" + str(horizon) + "/" + str(2)
    fca_data = read_max_avg_plan_time(path, n_veh)
    path = "./data_seed_" + seed +"/" + str(horizon) + "/" + str(1)
    random_data = read_max_avg_plan_time(path, n_veh)
    path = "./data_seed_" + seed +"/" + str(horizon) + "/" + str(0)
    static_data = read_max_avg_plan_time(path, 11)

    fig1, ax1 = plt.subplots()

    ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax1.set_xlabel(r'Number of vehicles $N_A$')
    ax1.set_ylabel(r'Duration of plan step $(\unit{\ms})$')
    ax1.plot(np.arange(1, n_veh+1) ,fca_data[0], '-C0s', fillstyle='none', label = r'max: $p_{\text{FCA}}$', linewidth=1, mew=1)
    ax1.plot(np.arange(1, n_veh+1) ,fca_data[1], '-C0d', fillstyle='none',label = r'median: $p_{\text{FCA}}$', linewidth=1)

    ax1.plot(np.arange(1, n_veh+1) ,random_data[0], '-C1s', fillstyle='none', label=r'max: $p_{r}$', linewidth=1)
    ax1.plot(np.arange(1, n_veh+1) ,random_data[1], '-C1d', fillstyle='none', label=r'median: $p_{r}$', linewidth=1)

    ax1.plot(np.arange(1, 11+1) ,static_data[0], '-C2s', fillstyle='none', label=r'max: $p_{s}$', linewidth=1)
    ax1.plot(np.arange(1, 11+1) ,static_data[1], '-C2d', fillstyle='none', label=r'median: $p_{s}$', linewidth=1)

    ax1.legend()
    plt.tight_layout()

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

latex_export()
#print(str(matplotlib.get_cachedir()))
#matplotlib.font_manager.findSystemFonts(fontpaths=None, fontext='ttf')

font = {#'family' : 'serif',
        #'serif'  : 'Times',
        'size'   : 8}

matplotlib.rc('font', **font)

#plot_avg_speed()
#plt.savefig('avg_speed.pgf')

#plot_prio_used()
#plt.savefig('new_prio_used.pgf')
#plot_average_speed_zoomed()
#plt.savefig('avg_speed_zoomed.pgf')

#plot_avg_speed()
#plt.savefig('avg_speed.pgf')


#plot_speed_comparison_h()
#plt.savefig('avg_speed_10s.pgf')

#plot_stop()
#plt.savefig('veh_until_infeasible.pgf')
plot_case_boxplot() # figure is saved in method

plot_stop_boxplot()
plt.savefig('veh_until_infeasible_boxplot.pgf')

plot_plan_time()
plt.savefig('plan_step_duration.pgf')
