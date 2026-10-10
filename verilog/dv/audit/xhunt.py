#!/usr/bin/env python3
"""Find the first time each dumped signal takes an unknown value after a given time.
Usage: python3 xhunt.py build/dump.vcd [after_ns]"""
import sys
from collections import defaultdict
path = sys.argv[1]; after = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
names = {}; scope = []; t = 0; first_x = {}; last_val = {}; seen_known = defaultdict(bool)
with open(path) as f:
    for line in f:
        line = line.strip()
        if not line: continue
        if line.startswith('$scope'): scope.append(line.split()[2])
        elif line.startswith('$upscope'): scope.pop()
        elif line.startswith('$var'):
            p = line.split(); names[p[3]] = '.'.join(scope[1:] + [p[4]])
        elif line[0] == '#': t = int(line[1:])
        elif line[0] in 'bB':
            v, code = line[1:].split(); name = names.get(code, code)
            if 'x' in v.lower():
                if t >= after and name not in first_x and seen_known[name]: first_x[name] = (t, v)
            else: seen_known[name] = True
        elif line[0] in '01xXzZ':
            v, code = line[0], line[1:]; name = names.get(code, code)
            if v in 'xX':
                if t >= after and name not in first_x and seen_known[name]: first_x[name] = (t, v)
            else: seen_known[name] = True
for name, (tt, v) in sorted(first_x.items(), key=lambda kv: kv[1][0])[:25]:
    print(f"{tt/1000:12.1f} ns  {name:45s} {v}")
print("signals going unknown after a known value:", len(first_x))
