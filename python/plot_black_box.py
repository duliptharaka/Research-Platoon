#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib
import pandas as pd
import sys
import glob
import warnings
warnings.filterwarnings("ignore")
#matplotlib.use('Agg')
plt.rcParams['figure.figsize'] = 12, 26

SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 12

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=MEDIUM_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE, loc='upper right')    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title


matplotlib.rcParams['lines.linewidth'] = 0.2

#matplotlib.use('Agg')
#matplotlib.style.use('ggplot')


def get_2ax(ax,d1, d2, d3, l1, l2, l3, y1, y2, title, df):
    ax1 = ax
    x = list(range(len(d1)))
    ax1.plot(x, d1, c='skyblue', zorder=0, label=l1)
    ax1.plot(x, d2, c='mediumblue',zorder=10, label=l2)
    ax1.plot([],[], 'r', label='front_crash')
    ax1.plot([],[], 'k', label='rear_ended')
    ax1.set_ylabel(y1)
    for i, c in enumerate(df['crashes']):
        if c == 1:
            ax1.axvline(x=i,color='r')

    for i, c in enumerate(df['rear_ended']):
        if c == 1:
            ax1.axvline(x=i,color='k',)
    ax2 = ax1.twinx()
    ax2.plot(x, d3, c='g', zorder=5, label=l3)
    ax2.set_ylabel(y2)
    lines =  ax1.get_lines() + ax2.get_lines()
    leg=ax1.legend(lines, [l.get_label() for l in lines],loc='center left', bbox_to_anchor=(1.1, 0.5))
    for legobj in leg.legendHandles:
        legobj.set_linewidth(2.0)
    ax1.set_xlabel('Time (0.1s)')
    ax1.set_title(title)



def get_1ax(ax,d1, d2, l1, l2, y1, title, df):
    ax1 = ax
    x = list(range(len(d1)))
    ax1.plot(x, d1, c='skyblue', zorder=0, label=l1)
    ax1.plot(x, d2, c='mediumblue',zorder=10, label=l2)
    ax1.plot([],[], 'r', label='front_crash')
    ax1.plot([],[], 'k', label='rear_ended')
    ax1.set_ylabel(y1)
    for i, c in enumerate(df['crashes']):
        if c == 1:
            ax1.axvline(x=i,color='r')

    for i, c in enumerate(df['rear_ended']):
        if c == 1:
            ax1.axvline(x=i,color='k',)
    lines =  ax1.get_lines()
    leg=ax1.legend(lines, [l.get_label() for l in lines],loc='center left', bbox_to_anchor=(1.1, 0.5))
    for legobj in leg.legendHandles:
        legobj.set_linewidth(2.0)
    ax1.set_xlabel('Time (0.1s)')
    ax1.set_title(title)





def _plot_black_box_middle(df):


    fig, axes = plt.subplots(nrows=13, ncols=1)
    axes_i = 0

    get_2ax(
        axes[axes_i],
        df['measured_front_distance']-df['real_front_distance'],
        df['estimated_front_distance']-df['real_front_distance'],
        df['real_front_distance'],
        'front_distance_measurement_error',
        'front_distance_estimation_error',
        'real_front_distance (Right)',
        'Distance (m)',
        'Distance (m)',
        'front_distance',
        df
        )

    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['measured_front_v_difference']-df['real_front_v_difference'],
        df['estimated_front_v_difference']-df['real_front_v_difference'],
        df['real_front_v_difference'],
        'front_v_difference_measurement_error',
        'front_v_difference_estimation_error',
        'real_front_v_difference (Right)',
        'Velocity (m/s)',
        'Velocity (m/s)',
        'front_v_difference',
        df
        )


    axes_i += 1
    get_1ax(
        axes[axes_i],
        df['front_measured_d']-df['front_real_d'],
        df['front_estimated_d']-df['front_real_d'],
        'front_position_measurement_error',
        'front_position_estimation_error',
        'Distance (m)',
        'front_position',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['front_measured_v']-df['front_real_v'],
        df['front_estimated_v']-df['front_real_v'],
        df['front_real_v'],
        'front_v_measurement_error',
        'front_v_estimation_error',
        'front_real_v (Right)',
        'Velocity (m/s)',
        'Velocity (m/s)',
        'front_v',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['front_measured_a']-df['front_real_a'],
        df['front_estimated_a']-df['front_real_a'],
        df['front_real_a'],
        'front_a_measurement_error',
        'front_a_estimation_error',
        'front_real_a (Right)',
        'Acceleration (m/s^2)',
        'Acceleration (m/s^2)',
        'front_a',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['measured_following_distance']-df['real_following_distance'],
        df['estimated_following_distance']-df['real_following_distance'],
        df['real_following_distance'],
        'following_distance_measurement_error',
        'following_distance_estimation_error',
        'real_following_distance (Right)',
        'Distance (m)',
        'Distance (m)',
        'following_distance',
        df
        )

    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['measured_following_v_difference']-df['real_following_v_difference'],
        df['estimated_following_v_difference']-df['real_following_v_difference'],
        df['real_following_v_difference'],
        'following_v_difference_measurement_error',
        'following_v_difference_estimation_error',
        'real_following_v_difference (Right)',
        'Velocity (m/s)',
        'Velocity (m/s)',
        'following_v_difference',
        df
        )

    axes_i += 1
    get_1ax(
        axes[axes_i],
        df['following_measured_d']-df['following_real_d'],
        df['following_estimated_d']-df['following_real_d'],
        'following_position_measurement_error',
        'following_position_estimation_error',
        'Distance (m)',
        'following_position',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['following_measured_v']-df['following_real_v'],
        df['following_estimated_v']-df['following_real_v'],
        df['following_real_v'],
        'following_v_measurement_error',
        'following_v_estimation_error',
        'following_real_v (Right)',
        'Velocity (m/s)',
        'Velocity (m/s)',
        'following_v',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['following_measured_a']-df['following_real_a'],
        df['following_estimated_a']-df['following_real_a'],
        df['following_real_a'],
        'following_a_measurement_error',
        'following_a_estimation_error',
        'following_real_a (Right)',
        'Acceleration (m/s^2)',
        'Acceleration (m/s^2)',
        'following_a',
        df
        )


    axes_i += 1
    get_1ax(
        axes[axes_i],
        df['measured_d']-df['real_d'],
        df['estimated_d']-df['real_d'],
        'position_measurement_error',
        'position_estimation_error',
        'Distance (m)',
        'position',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['measured_v']-df['real_v'],
        df['estimated_v']-df['real_v'],
        df['real_v'],
        'v_measurement_error',
        'v_estimation_error',
        'real_v (Right)',
        'Velocity (m/s)',
        'Velocity (m/s)',
        'velocity',
        df
        )


    axes_i += 1
    get_2ax(
        axes[axes_i],
        df['measured_a']-df['real_a'],
        df['estimated_a']-df['real_a'],
        df['real_a'],
        'a_measurement_error',
        'a_estimation_error',
        'real_a (Right)',
        'Acceleration (m/s^2)',
        'Acceleration (m/s^2)',
        'acceleration',
        df
        )

    fig.tight_layout()
    return fig

def show_plt_black_box(df):
    fig = _plot_black_box_middle(df)
    fig.tight_layout()
    fig.show()

def save_plt_black_box(pdf_fname, df):
    fig = _plot_black_box_middle(df)
    fig.tight_layout()
    fig.savefig(pdf_fname)
    plt.close(fig)

def csv_to_pdf(csv_fname):
    df=pd.read_csv(csv_fname,delim_whitespace=True)
    save_plt_black_box(csv_fname[:-4]+'.pdf',df)


if __name__ == '__main__':
    for files in (sys.argv[1:]):
        for file in glob.glob(files):
            try:
                print('plot',file)
                csv_to_pdf(file)
            except Exception as e:
                print('Cannot plot',file, e)
                raise e
    input('Press any key to exit...')

#def csv_to_plt(fname):
#    df = pd.read_csv('d_no_filter_v3_lie_to_v2_dix_var_measured_velocity_at_t_0_7_0.5.csv',delim_whitespace=True)
#    fig = _plot_black_box(df)
#    fig.show()


#if __name__ == '__main__':
#    csv_to_plt('d_no_filter_v3_lie_to_v2_dix_var_measured_velocity_at_t_0_7_0.5.csv')
#    input('Press any key to exit...')