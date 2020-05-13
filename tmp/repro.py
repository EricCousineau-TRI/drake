from robotlocomotion import quaternion_t

from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmInterfaceSystem, LcmSubscriberSystem


class PrintTime(LeafSystem):
    def DoPublish(self, context, events):
        print(f"t: {context.get_time()}")


def main():
    lcm = DrakeLcm("memq://")
    lcm_type = quaternion_t
    channel = "JUNK"
    dt_sec = 1.

    builder = DiagramBuilder()
    builder.AddSystem(PrintTime()).DeclarePeriodicPublish(dt_sec)
    lcm_system = builder.AddSystem(LcmInterfaceSystem(lcm))
    builder.AddSystem(LcmSubscriberSystem.Make(channel, lcm_type, lcm))
    diagram = builder.Build()

    for do_publish in (False, True):
        print(f"--- do_publish: {do_publish} ---")
        simulator = Simulator(diagram)
        if do_publish:
            lcm.Publish(channel, lcm_type().encode())
        print("Initialize")
        simulator.Initialize()
        print("AdvanceTo")
        simulator.AdvanceTo(dt_sec)
        print()


if __name__ == "__main__":
    main()

"""
--- do_publish: False ---
Initialize
t: 0.0
AdvanceTo
t: 1.0

--- do_publish: True ---
Initialize
AdvanceTo
t: 1.0
"""
