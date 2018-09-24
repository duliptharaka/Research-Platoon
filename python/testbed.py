#!/usr/bin/env python3
from noises import Noises
from simulator import Simulator
from plot_black_box import show_plt_black_box, save_plt_black_box
import time
import numpy as np
import scipy.stats as st
from copy import deepcopy
import socket
import pandas as pd
import multiprocessing as mp
import os

def m_v_ci(data, confidence=0.95):
    a = 1.0*np.array(data)
    n = len(a)
    m, se = np.mean(a), st.sem(a)
    ci = se * st.t.ppf((1+confidence)/2., n-1)
    v = np.var(a)
    return m, v, ci

def now():
    return time.strftime('%H:%M:%S',time.localtime())


class TestBed():
    """Run multiple times of the same simulation and get confident interval
    Using multiprocessing

    Modified to get results from the second following car in the platoon
    """
    def __init__(self, simulator_args):
        self.simulator_args = simulator_args

    def _run(self, steps, results_type='platoon'):
        simulator = Simulator(*self.simulator_args)
        simulator.run(steps)
        if results_type=='v2':
            mpg,crash = simulator.platoon.vehicles[2].get_result()
        elif results_type=='platoon':
            mpg,crash = simulator.platoon.get_result()
        black_boxs = []
        for v in simulator.platoon.vehicles[1:4]:
            black_boxs.append(dict(v.black_box))
        return mpg,crash,black_boxs

    def run(self, steps=6990, epochs=50, sample_file_name=None, pool=None, results_type='platoon'):

        if pool is None:
            pool = mp.Pool(min(epochs,mp.cpu_count())+2)
            close_pool = True
        else:
            close_pool = False
        jobs = [None]*epochs
        mpg_results = [None]*epochs
        crash_results = [None]*epochs
        sample_black_boxs = [None]*epochs

        for i in range(epochs):
            jobs[i] = pool.apply_async(self._run, (steps,results_type,))

        for i in range(epochs):
            mpg_results[i], crash_results[i], sample_black_boxs[i]=jobs[i].get() #
            #print('get',i)


        jobs = []

        jobs.append(pool.apply_async(m_v_ci, (mpg_results,)))
        jobs.append(pool.apply_async(m_v_ci, (crash_results,)))


        if sample_file_name:
            mc = max(crash_results)
            if mc > 0:
                most_interested = crash_results.index(mc)
            else:
                most_interested = mpg_results.index(min(mpg_results))
            for i, black_box in enumerate(sample_black_boxs[most_interested]):
                #print(type(black_box))
                data = pd.DataFrame(black_box)
                #jobs.append(pool.apply_async(save_plt_black_box,(sample_file_name+'_v'+str(i+1)+'.pdf',data)))
                jobs.append(pool.apply_async(data.to_csv,(sample_file_name+'_v'+str(i+1)+'.csv','\t', None, '%.4f')))


        mpg_m,mpg_v,mpg_ci=jobs[0].get()
        crash_m,crash_v,crash_ci=jobs[1].get()
        for job in jobs[2:]:
            job.get()
        
        if close_pool:
            pool.close()
        return mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci


def example(sample_file_name='example',steps=6990,epochs=50):
    simulator_args = (
    'c', # 'c' ,'d'
    [
        Noises([0.]*21,[0.0]*15+[0.]*6),
        Noises([0.]*21,[0.0]*15+[0.]*6),
        Noises([0.]*21,[0.0]*15+[0.]*6),
        Noises([0.]*12,[0.0]*9+[0.]*3)
    ],
    [
        Noises([0.],[0.5]),
        Noises([0.],[0.5]),
        Noises([0.],[0.5]),
        Noises([0.],[0.5])
    ],
    'ukf',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
    'first_car_a10',
    'first_car_v',
    0.1,
    1)
    print(now(),'start',str(epochs),'epochs')
    tb =TestBed(simulator_args)
    mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci = tb.run(steps,epochs,sample_file_name)
    print(now(),mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci)


def base_case(steps=6990,epochs=50):
    pool = mp.Pool(min(50,mp.cpu_count())+2)
    hostname = socket.gethostname()
    hostid = int(hostname[4])-2

    simulator_args_template = [
    'd', # 'c' ,'d'
    [
        Noises([0.]*21,[0.]*15+[0.]*6),
        Noises([0.]*21,[0.]*15+[0.]*6),
        Noises([0.]*21,[0.]*15+[0.]*6),
        Noises([0.]*12,[0.]*9+[0.]*3)
    ],
    [
        Noises([0.],[0.]),
        Noises([0.],[0.]),
        Noises([0.],[0.]),
        Noises([0.],[0.])
    ],
    'no_filter',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
    'first_car_a10',
    'first_car_v',
    0.1,
    1]


    job_id = 0
    for controller_type in ['c','d']:
        for distance in [100,1]:
            if np.rint(job_id%8) == hostid:
                simulator_args = deepcopy(simulator_args_template)
                simulator_args[0] = controller_type
                simulator_args[7] = distance

    

                tb =TestBed(simulator_args)
                mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci = tb.run(steps=6990,epochs=50,pool=pool,results_type='platoon')

                print(now(),mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci)
    
                add_to_csv('08312018_base_line.csv','\t'.join([str(v) for v in [controller_type, str(distance), mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci]]))
            job_id += 1
    
    pool.close()




def add_to_csv(file_name, new_line):
    with open(file_name, 'a') as f:
        f.write(new_line+'\n')

def single_lie_run_on_cluster_one_victim():
    pool = mp.Pool(min(50,mp.cpu_count())+2)
    hostname = socket.gethostname()
    hostid = int(hostname[4])-2

    simulator_args_template = [
    'd', # 'c' ,'d'
    [
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*12,[0.1]*9+[0.]*3)
    ],
    [
        Noises([0.],[0.01]),
        Noises([0.],[0.01]),
        Noises([0.],[0.01]),
        Noises([0.],[0.01])
    ],
    'ukf',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
    'first_car_a10',
    'first_car_v',
    0.1,
    1]

    v1_lie_to_v2 = {
                    'measured_position_at_t_0':0,
                    'measured_velocity_at_t_0':1,
                    'measured_acceleration_at_t_0':2,
                    'measured_distance_at_t_0':9,
                    'measured_velocity_difference_at_t_0':10,
                    'measured_acceleration_difference_at_t_0':11,
                    'announced_position_at_t_-1':15,
                    'announced_velocity_at_t_-1':16,
                    'announced_acceleration_at_t_-1':17
                    }

    v3_lie_to_v2 = {
                    'measured_position_at_t_0':6,
                    'measured_velocity_at_t_0':7,
                    'measured_acceleration_at_t_0':8,
                    'measured_distance_at_t_0':12,
                    'measured_velocity_difference_at_t_0':13,
                    'measured_acceleration_difference_at_t_0':14,
                    'announced_position_at_t_-1':18,
                    'announced_velocity_at_t_-1':19,
                    'announced_acceleration_at_t_-1':20
                    }

    types = {
        'sigma':[0.2,1],
        'mean':[-1, -0.2, 0.2, 1]
    }

    malicious_vehicles = {
        'v1_lie_to_v2':v1_lie_to_v2,
        'v3_lie_to_v2':v3_lie_to_v2
    }


    job_id = 0
    for filter_type in ['ukf', 'no_filter', 'average', 'average_without_high_low']:
        for controller_type in ['c','d']:
            for malicious_vehicle in malicious_vehicles:
                for sigma_mean in types:
                    for value in types[sigma_mean]:
                        for lie in malicious_vehicles[malicious_vehicle]:
                            if np.rint(job_id%8) == hostid:
                                simulator_args = deepcopy(simulator_args_template)
                                simulator_args[0] = controller_type
                                simulator_args[3] = filter_type
                                getattr(simulator_args[1][1],sigma_mean)[malicious_vehicles[malicious_vehicle][lie]]=value
                                file_name = '_'.join([filter_type, controller_type,malicious_vehicle,sigma_mean,lie,str(value)])
    
    
                                print(now(),file_name)
                                tb =TestBed(simulator_args)
                                mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci = tb.run(sample_file_name=file_name, steps=6990,epochs=50,pool=pool,results_type='v2')
                                print(now(),mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci)
        
                                add_to_csv('07162018_massive.csv','\t'.join([str(v) for v in [filter_type, controller_type,malicious_vehicle,sigma_mean,lie,str(value), mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci]]))
                            job_id += 1
    
    pool.close()


def single_lie_run_on_cluster_one_malicious():
    pool = mp.Pool(min(50,mp.cpu_count())+2)
    hostname = socket.gethostname()
    hostid = int(hostname[4])-2

    simulator_args_template = [
    'd', # 'c' ,'d'
    [
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*12,[0.1]*9+[0.]*3)
    ],
    [
        Noises([0.],[0.01]),
        Noises([0.],[0.01]),
        Noises([0.],[0.01]),
        Noises([0.],[0.01])
    ],
    'ukf',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
    'first_car_a10',
    'first_car_v',
    0.1,
    1]


    v2_lie_to_v1 = {
                    'measured_position_at_t_0':6,
                    'measured_velocity_at_t_0':7,
                    'measured_acceleration_at_t_0':8,
                    'measured_distance_at_t_0':12,
                    'measured_velocity_difference_at_t_0':13,
                    'measured_accleration_difference_at_t_0':14,
                    'announced_position_at_t_-1':18,
                    'announced_velocity_at_t_-1':19,
                    'announced_acceleration_at_t_-1':20
                    }

    v2_lie_to_v3 = {
                    'measured_position_at_t_0':0,
                    'measured_velocity_at_t_0':1,
                    'measured_acceleration_at_t_0':2,
                    'measured_distance_at_t_0':9,
                    'measured_velocity_difference_at_t_0':10,
                    'measured_accleration_difference_at_t_0':11,
                    'announced_position_at_t_-1':15,
                    'announced_velocity_at_t_-1':16,
                    'announced_acceleration_at_t_-1':17
                    }


    types = {
        'sigma':[0.2,1],
        'mean':[-1, -0.2, 0.2, 1]
    }

    malicious_vehicles = {
        'v2_lie_to_v1':v2_lie_to_v1,
        'v2_lie_to_v3':v2_lie_to_v3
    }
    victim_vehicles = {
        'v2_lie_to_v1':0,
        'v2_lie_to_v3':2
    }


    job_id = 0
    for filter_type in ['ukf']:#, 'no_filter', 'average', 'average_without_high_low']:
        for controller_type in ['c','d']:
            for malicious_vehicle in malicious_vehicles:
                for sigma_mean in types:
                    for value in types[sigma_mean]:
                        for lie in malicious_vehicles[malicious_vehicle]:
                            if np.rint(job_id%8) == hostid:
                                simulator_args = deepcopy(simulator_args_template)
                                simulator_args[0] = controller_type
                                simulator_args[3] = filter_type
                                getattr(simulator_args[1][victim_vehicles[malicious_vehicle]],sigma_mean)[malicious_vehicles[malicious_vehicle][lie]]=value
                                file_name = '_'.join([filter_type, controller_type,malicious_vehicle,sigma_mean,lie,str(value)])
    
    
                                print(now(),file_name)
                                tb =TestBed(simulator_args)
                                mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci = tb.run(sample_file_name=file_name, steps=6990,epochs=50,pool=pool,results_type='platoon')
                                print(now(),mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci)
        
                                add_to_csv('08082018_massive.csv','\t'.join([str(v) for v in [filter_type, controller_type,malicious_vehicle,sigma_mean,lie,str(value), mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci]]))
                            job_id += 1
    
    pool.close()


def poor_car_run_on_cluster_one_malicious():
    pool = mp.Pool(min(50,mp.cpu_count())+2)
    hostname = socket.gethostname()
    hostid = int(hostname[4])-2

    simulator_args_template = [
    'd', # 'c' ,'d'
    [
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*21,[0.1]*15+[0.]*6),
        Noises([0.]*12,[0.1]*9+[0.]*3)
    ],
    [
        Noises([0.],[0.01]),
        Noises([0.],[0.01]),
        Noises([0.],[0.01]),
        Noises([0.],[0.01])
    ],
    'ukf',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
    'first_car_a10',
    'first_car_v',
    0.1,
    1]


    types = {
        'sigma':[0.2,1],
        'mean':[-1,-0.5,-0.2,-0.1,0.1,0.2, 0.5,1]
    }


    poor_vehicle = 1


    job_id = 0
    for filter_type in ['no_filter', 'average', 'average_without_high_low']:#'ukf']:#, 
        for controller_type in ['c','d']:
                for sigma_mean in types:
                    for value in types[sigma_mean]:
                            if np.rint(job_id%8) == hostid:
                                simulator_args = deepcopy(simulator_args_template)
                                simulator_args[0] = controller_type
                                simulator_args[3] = filter_type
                                getattr(simulator_args[2][poor_vehicle],sigma_mean)[0]=value
                                file_name = '_'.join([filter_type, controller_type,str(poor_vehicle),sigma_mean,'actuator',str(value)])
    
    
                                print(now(),file_name)
                                tb =TestBed(simulator_args)
                                mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci = tb.run(sample_file_name=file_name, steps=6990,epochs=50,pool=pool,results_type='platoon')
                                print(now(),mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci)
        
                                add_to_csv('08272018_massive.csv','\t'.join([str(v) for v in [filter_type, controller_type,str(poor_vehicle),sigma_mean,'actuator',str(value), mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci]]))
                            job_id += 1
    
    pool.close()



def noise_vs_filter_run_on_cluster():
    pool = mp.Pool(min(50,mp.cpu_count())+2)
    hostname = socket.gethostname()
    hostid = int(int(hostname[4])-2)

    simulator_args_template = [
    'd', # 'c' ,'d'
    [
        Noises([0.]*21,[0.]*15+[0.]*6),
        Noises([0.]*21,[0.]*15+[0.]*6),
        Noises([0.]*21,[0.]*15+[0.]*6),
        Noises([0.]*12,[0.]*9+[0.]*3)
    ],
    [
        Noises([0.],[0.]),
        Noises([0.],[0.]),
        Noises([0.],[0.]),
        Noises([0.],[0.])
    ],
    'ukf',    # 'ukf', 'no_filter', 'average', 'average_without_high_low'
    'first_car_a10',
    'first_car_v',
    .1,
    1]

    types = ['actuator']#['sensor',

    job_id = 0
    for filter_type in ['ukf']:#['no_filter', 'ukf', 'average', 'average_without_high_low']:
        for controller_type in ['c', 'd']:
            for controller_actuator in types:
                for value in [0.,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.41,0.42,0.43,0.44,0.45,0.46,0.47,0.48,0.49]:
                    #print(job_id%8,hostid)
                    if job_id%8 == hostid:
                        #print('working...')
                        simulator_args = deepcopy(simulator_args_template)
                        simulator_args[0] = controller_type
                        simulator_args[3] = filter_type
                        if controller_actuator== 'sensor':
                            simulator_args[1] = [
                                                    Noises([0.]*21,[value]*15+[0.]*6),
                                                    Noises([0.]*21,[value]*15+[0.]*6),
                                                    Noises([0.]*21,[value]*15+[0.]*6),
                                                    Noises([0.]*12,[value]*9+[0.]*3)
                                                ]
                        elif controller_actuator== 'actuator':
                            simulator_args[2] = [
                                                    Noises([0.],[value]),
                                                    Noises([0.],[value]),
                                                    Noises([0.],[value]),
                                                    Noises([0.],[value])
                                                ]    
                        task_name = '_'.join([filter_type, controller_type,controller_actuator,str(value)])
    
    
                        print(now(),task_name)
                        tb =TestBed(simulator_args)
                        mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci = tb.run(sample_file_name=None, steps=6990,epochs=50,pool=pool,results_type='platoon')
                        print(now(),mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci)
                        file_name = '_'.join([controller_type, filter_type,controller_actuator]) +'_base.csv'
                        print('worite to file:',file_name)
                        add_to_csv(file_name,'\t'.join([str(v) for v in [value, mpg_m,mpg_v,mpg_ci,crash_m,crash_v,crash_ci]]))
                    job_id += 1
    
    pool.close()


if __name__ == '__main__':
    base_case()
    #single_lie_run_on_cluster()
    #example('c_ukf_base_actuator',6990,50)
    #noise_vs_filter_run_on_cluster()
    #single_lie_run_on_cluster_one_malicious()
    #poor_car_run_on_cluster_one_malicious()
