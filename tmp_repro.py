import sys
sys.path.insert(0, "/home/eacousineau/proj/tri/repo/branches/drake/master/drake/.venv/lib/python3.8/site-packages")

import numpy as np
from pydrake.all import MakeVectorVariable
v = MakeVectorVariable(2, "v")
print(np.__file__)
print(np.__version__)
print(np.eye(2) @ v)
