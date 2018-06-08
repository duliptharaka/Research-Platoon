#!/usr/bin/env python3

import sys

def file_reader(file):
    with open(file, 'r') as f:
        for line in f:
            yield float(line)
    yield None

def file_writer(file, data):
    with open(file, 'a') as f:
        for line in data:
            f.write(str(line))
            f.write('\n')

def interpolate(x=10):
    fr = file_reader(sys.argv[1])
    data0 = next(fr)
    data1 = next(fr)
    while data1 is not None:
        delta = data1 - data0
        l = [data0+delta*i/x for i in range(x)]
        file_writer(sys.argv[2], l)
        data0 = data1
        data1 = next(fr)

if __name__ == '__main__':
    interpolate()