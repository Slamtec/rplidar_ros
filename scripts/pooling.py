#!/usr/bin/python
from numpy import array, mean, inf, isfinite
from numba import jit


@jit(nopython=True)
def max_pooling(input_array, value, array_len, function=max):
    return [function(input_array[pos:pos + value]) for pos in range(0, array_len, value)]


@jit(nopython=True)
def min_pooling(input_array, value, array_len, function=min):
    return [function(input_array[pos:pos + value]) for pos in range(0, array_len, value)]


@jit(nopython=True)
def mean_pooling(input_array, value, array_len, function=mean):
    return [function(input_array[pos:pos + value]) for pos in range(0, array_len, value)]


@jit(nopython=True)
def max_zwi_pooling(input_array, value, array_len):
    return [max(input_array[pos:pos + value][isfinite(input_array[pos:pos + value])])
            if isfinite(input_array[pos:pos + value]).any() else inf for pos in range(0, array_len, value)]


@jit(nopython=True)
def min_zwi_pooling(input_array, value, array_len):
    return [min(input_array[pos:pos + value][isfinite(input_array[pos:pos + value])])
            if isfinite(input_array[pos:pos + value]).any() else inf for pos in range(0, array_len, value)]


@jit(nopython=True)
def mean_zwi_pooling(input_array, value, array_len):
    return [mean(input_array[pos:pos + value][isfinite(input_array[pos:pos + value])])
            if isfinite(input_array[pos:pos + value]).any() else inf for pos in range(0, array_len, value)]


@jit(nopython=False)
def function_pooling(input_array, value, array_len, function):
    return [function(input_array[pos:pos + value]) for pos in range(0, array_len, value)]


@jit(nopython=False)
def zwi_function_pooling(input_array, value, array_len, function):  # zero weight inf
    return [function(input_array[pos:pos + value][isfinite(input_array[pos:pos + value])])
            if isfinite(input_array[pos:pos + value]).any() else inf for pos in range(0, array_len, value)]


@jit(nopython=True)
def position_pooling(input_array, position, sections):
    return [input_array[pos + position - 1] for pos in sections]


def pooling(input_array, value, method=None, zero_weight_inf=True, use_jit=True):
    array_len = len(input_array)
    if array_len % value:
        raise TypeError('input_array length % pooling value > 0')

    if type(method) is int:
        if method > value:
            raise TypeError('method > value')
        if use_jit:
            return position_pooling(input_array, value, array_len, method)
        return [input_array[pos + method - 1] for pos in range(0, array_len, value)]

    if zero_weight_inf:
        if use_jit:
            if method is max:
                return max_zwi_pooling(input_array, value, array_len)
            if method is min:
                return min_zwi_pooling(input_array, value, array_len)
            if method is mean:
                return mean_zwi_pooling(input_array, value, array_len)

        return zwi_function_pooling(input_array, value, array_len, function=method)
    if use_jit:
        if method is max:
            return max_pooling(input_array, value, array_len)
        if method is min:
            return min_pooling(input_array, value, array_len)
        if method is mean:
            return mean_pooling(input_array, value, array_len)
    return function_pooling(input_array, value, array_len, function=method)


if __name__ == '__main__':
    from functools import wraps
    from time import process_time

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
        return position_pooling(a, 2, 3200, 1)

    a = array(200 * [20.11221, 200.233525, -5.52335, 23.43434, -0.00976, inf, 119.2, 100.0, 120.12, 32.43, inf, inf, -120.34, 12.56565, 9.78665, 23.0890])

    for i in range(1, 10):
        pooled = test()
        # print(pooled)
        print(f'input_len: [{len(a)}], output_len: [{len(pooled)}]')
