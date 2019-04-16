#!/usr/bin/env python

import requests
import json
import urllib2
import IPython
import os
import time
from datetime import datetime
from matplotlib import pyplot as plt
from collections import OrderedDict
import matplotlib.dates as mdates
import rospy
import rospkg
import pandas as pd



experiment_dir = "/experiments/"
fmt = "%Y-%m-%dT%H:%M:%S"

class Experiment:
    timestamp = None
    scenario = None
    strategy = None
    exec_cost = None
    pareto_weight = None
    succeeded = True


def get_scenarios(experiments):
    """ Returns a set of all scenarios in the experiments"""
    return {exp.scenario for exp in experiments}

def plot_data(all_experiments, save_path):
    for scenario in get_scenarios(all_experiments):
        exps = [exp for exp in all_experiments if exp.scenario == scenario]
        plot_scenario(exps, save_path)
    

def plot_scenario(experiments, save_path):

    """Plots a list of experiments all belonging to the same scenario"""

    experiments.sort(key=lambda e:e.strategy)

    series = pd.Series([e.exec_cost * e.succeeded for e in experiments])
    ax = series.plot(kind='bar')
    
    ax.set_title(experiments[0].scenario)
    
    x_labels = [e.strategy for e in experiments]
    ax.set_xticklabels(x_labels)
    plt.tight_layout()
    plt.show()
    ax.get_figure().savefig(save_path + experiments[0].scenario + ".png")




def load_file(filepath, filename):
    exp = Experiment()
    with open(filepath + filename) as f:
        exp.timestamp = f.readline()
        line = f.readline()
        while line:
            parts = line.split()
            if len(parts) == 0:
                line = f.readline()
                continue
            
            if parts[0] == "ExecutionCost":
                exp.exec_cost = float(parts[2])
            elif parts[0] == "Strategy:":
                exp.strategy = parts[1]
            elif parts[0] == "Scenario:":
                exp.scenario = parts[1]
            elif parts[0] == "Action_Limit_Exceeded":
                exp.succeeded = False
            line = f.readline()
    
    # IPython.embed()
    return exp
        

def load_all_files():
    path = rospkg.RosPack().get_path('blindfolded_traveler_2D') + experiment_dir
    experiments = []
    for name in os.listdir(path):
        if name.endswith(".png") or\
           name.endswith(".pdf"):
           continue;
        experiments.append(load_file(path, name))
    plot_data(experiments, path)
    


if __name__=='__main__':
    rospy.init_node("plot_experiments")
    load_all_files();
