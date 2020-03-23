# from https://drakedevelopers.slack.com/archives/C2CK4CWE7/p1584994164030500

from pydrake.systems.primitives import ConstantVectorSource

source = ConstantVectorSource([None])
context = source.CreateDefaultContext()
y = source.get_output_port(0).Eval(context)
print(y)
