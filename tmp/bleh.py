# from https://drakedevelopers.slack.com/archives/C2CK4CWE7/p1584994164030500

from pydrake.systems.primitives import ConstantVectorSource


def main():
    source = ConstantVectorSource([None])
    context = source.CreateDefaultContext()
    y = source.get_output_port(0).Eval(context)
    print(y)


# main()
import sys, trace
sys.stdout = sys.stderr
tracer = trace.Trace(trace=1, count=0, ignoredirs=["/usr", sys.prefix])
tracer.runfunc(main)
