from math import sqrt
from sys import stdout

f = stdout

f.write('uint16_t Q16_RSQRT_LUT[] = {\n')
N = 192
for i in range(N):
    x = 1.0 + 3.0 * i/N
    f.write(hex(int(round(32768.0 / sqrt(x)))))
    if i != N-1:
        f.write(',')
    if i % 8 == 7:
        f.write('\n')
    else:
        f.write(' ')
f.write('};\n')

