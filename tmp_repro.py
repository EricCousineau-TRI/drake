import numpy as np
from pydrake.all import MakeVectorVariable
v = MakeVectorVariable(2, "v")
np.eye(2) @ v
