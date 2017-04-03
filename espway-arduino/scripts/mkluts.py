from math import sqrt

with open('../include/q16_luts.h', 'w') as f:
    f.write('uint16_t Q16_RSQRT_LUT[] = {\n')
    N = 192
    for i in range(192):
        x = 1 + 3 * i/N
        f.write(hex(round(32768 / sqrt(x))))
        if i != N-1:
            f.write(',')
        if i % 8 == 7:
            f.write('\n')
        else:
            f.write(' ')
    f.write('};\n')

