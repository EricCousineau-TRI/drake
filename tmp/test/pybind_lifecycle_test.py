"""
Tests for pybind lifecycle issues.

DO NOT USE unless you acknowledge that this may cause segfaults that you are
responsible for debugging.
"""

import gc
import unittest
import weakref

from pydrake.systems.framework import DiagramBuilder, LeafSystem

from drake.tmp.pybind_lifecycle import ClearPatients, GetPatients


def make_diagram():

    def inner():
        builder = DiagramBuilder()
        system = LeafSystem()
        system.set_name("system")
        builder.AddSystem(system)
        diagram = builder.Build()
        # Return references so that things should go out of scope normally, but
        # will not due to reference cycles.
        # The references cycles produced here are mentioned in the following
        # Anzu issue; see the issue for linked Drake isues:
        # https://github.shared-services.aws.tri.global/robotics/anzu/issues/13065
        # Note: The weakref's here are generally *only* for testing purposes.
        # If your code has direct control over the lifetime, you should not
        # use weakref's.
        return weakref.ref(builder), weakref.ref(diagram), weakref.ref(system)

    refs = inner()
    # Garbage collect to prove cycles keep items alive.
    gc.collect()
    return refs


class Test(unittest.TestCase):
    def assertListIs(self, a, b):
        a_ids = [id(a_i) for a_i in a]
        b_ids = [id(b_i) for b_i in b]
        self.assertEqual(a_ids, b_ids, f"{a} != {b}")

    def test_make_diagram(self):
        builder_ref, diagram_ref, system_ref = make_diagram()
        # Dereference weakref's.
        builder = builder_ref()
        diagram = diagram_ref()
        system = system_ref()
        # Reference cycle prevents objects from being GC'd.
        self.assertIsNotNone(builder)
        self.assertIsNotNone(diagram)
        self.assertIsNotNone(system)

    def test_get_patients(self):
        builder_ref, diagram_ref, system_ref = make_diagram()
        # Dereference weakref's.
        builder = builder_ref()
        diagram = diagram_ref()
        system = system_ref()
        # As shown here, the cycle is formed between `builder` and `system`.
        self.assertListIs(GetPatients(builder), [system, diagram])
        self.assertListIs(GetPatients(diagram), [])
        self.assertListIs(GetPatients(system), [builder])

    def test_clear_patients(self):
        builder_ref, diagram_ref, system_ref = make_diagram()
        # Clear cycles for builder.
        # WARNING: This may cause use-after-free errors. Use this with caution!
        # Notes:
        # - Depending on how your code operates, and what accessor you use,
        #   you may need to clear patients on other objects as well.
        # - It be difficult to free everything if you have sub-builders /
        #   diagrams that are constructed in Python.
        # - Lifetime cycles may not occur if `builder.Build()` is called in
        #   C++.
        ClearPatients(builder_ref())
        # Now objects are GC'd.
        self.assertIsNone(builder_ref())
        self.assertIsNone(diagram_ref())
        self.assertIsNone(system_ref())


if __name__ == "__main__":
    unittest.main()
