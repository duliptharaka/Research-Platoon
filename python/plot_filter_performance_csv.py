#!/usr/bin/env python3


import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import pandas as pd
import sys
import glob

def plot_box(csv_fname):
    

    df=pd.read_csv(csv_fname,delim_whitespace=True, header=None,index_col=0)
    df=df.sort_index()
    #print(df)
    d ={
    'c': 'Centralized controller, ',
    'd': 'Decentralized controller, ',
    'average':'average of all input',
    'low':'average without high and low',
    'ukf':'Kalman Filter',
    'filter':'no filter'
    }
    fields = csv_fname[:-4].split('_')
    title = d[fields[0]]+d[fields[-3]]


    pdf_fname=csv_fname[:-4]+'.pdf'


    fig = plt.figure(figsize=(6, 6))
    plt.ion()
    plt.show()
    x= list(range(50))
    x = [0.00,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.41,0.42,0.43,0.44,0.45,0.46,0.47,0.48,0.49]

    plt.subplot(211)
    plt.title(title)
    plt.errorbar(x, df[1],df[3],linewidth=1, capsize=2)
    if 'actuator' in csv_fname:
        plt.xlabel("Actuator Noise Level")
    elif 'sensor' in csv_fname:
        plt.xlabel("Sensor Noise Level")
    plt.ylabel("Fuel Consumption (MPG)")

    plt.subplot(212)
    plt.errorbar(x, df[4],df[6],linewidth=1, capsize=2)
    if 'actuator' in csv_fname:
        plt.xlabel("Actuator Noise Level")
    elif 'sensor' in csv_fname:
        plt.xlabel("Sensor Noise Level")
    plt.ylabel("Crashes")


    plt.tight_layout()

    fig.savefig(pdf_fname)
    plt.close()
    #plt.savefig(sys.argv[2],bbox_inches='tight', dpi=600)

    # basic plot
    #plt.boxplot(data)


    #axes_i = 0

if __name__ == '__main__':
    for files in (sys.argv[1:]):
        for file in glob.glob(files):
            try:
                print('plot',file)
                plot_box(file)
            except Exception as e:
                print('Cannot plot',file, e)
                raise e
    input('Press any key to exit...')