#!/usr/bin/env python3
from math import isinf, inf
from numpy import empty
from numba import jit


@jit(nopython=True)
def resample(data, new_size):
    size = data.size
    result_data = empty(new_size, data.dtype)
    carry = 0
    old_pos = 0
    for new_pos in range(new_size):
        bill = carry
        while old_pos*new_size - new_pos*size < size:
            if isinf(data[old_pos]):
                bill = inf
                old_pos += 1
                continue
            bill += data[old_pos]
            old_pos += 1
        if isinf(bill):
            result_data[new_pos] = bill
            continue
        carry = (old_pos-(new_pos+1)*size/new_size)*data[old_pos-1]
        bill -= carry
        result_data[new_pos] = bill*new_size/size
    return result_data


if __name__ == '__main__':
    from functools import wraps
    from time import process_time
    from numpy import array

    def timing(function):
        @wraps(function)
        def wrap(*args, **kw):
            ts = process_time()
            result = function(*args, **kw)
            print(f'[{function.__name__}] took: {1000 * (process_time() - ts)} ms')
            return result

        return wrap

    @timing
    def test():
        return resample(a, 1600)

    a = array(200 * [20.11221, 200.233525, -5.52335, 23.43434, -0.00976, inf, 119.2, 100.0, 120.12, 32.43, inf, inf, -120.34, 12.56565, 9.78665, 23.0890])

    for i in range(1, 10):
        resampled = test()
        # print(resampled)
        print(f'input_len: [{len(a)}], output_len: [{len(resampled)}]')
