#!/usr/bin/env python

import math
import numpy as np
import sympy as sp


if __name__ == "__main__":
    x, y, yaw, v, w, dt = sp.symbols('x y yaw v w dt')
    eq_1 = x + (v / w) * (sp.cos(yaw+w*dt) - sp.cos(yaw))
    z = sp.diff(eq_1, w)
    
    print(z)
