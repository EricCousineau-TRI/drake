#!/usr/bin/env python

from __future__ import print_function
import argparse
import copy
import os
from Queue import Queue
import select
import sys
from threading import Thread, Lock

import numpy as np
# Hacky, but this is the simplest route right now.
# @ref https://www.datadoghq.com/blog/engineering/protobuf-parsing-in-python/
from google.protobuf.internal.decoder import _DecodeVarint32

from drake.common.proto.matlab_rpc_pb2 import MatlabArray, MatlabRPC


class _ExecutionInProgress(object):
    def __init__(self):
        self.count = 0

    def push(self):
        self.count += 1

    def pop(self):
        assert self.count > 0
        self.count -= 1


def _get_required_helpers(scope_locals):
    # Provides helpers to keep C++ interface as simple as possible.
    # @returns Dictionary containing the helpers needed.
    def getitem(obj, index):
        """Global function for `obj[index]`. """
        return obj[index]

    def setitem(obj, index, value):
        """Global function for `obj[index] = value`. """
        obj[index] = value
        return obj[index]

    def call(obj, *args, **kwargs):
        return obj(*args, **kwargs)

    def pass_through(value):
        """Pass-through for direct variable access. """
        return value

    def make_tuple(*args):
        """Create a tuple from an argument list. """
        return tuple(args)

    def make_list(*args):
        """Create a list from an argument list. """
        return list(args)

    def make_kwargs(*args):
        """Create a keyword argument object from an argument list. """
        assert len(args) % 2 == 0
        keys = args[0::2]
        values = args[1::2]
        kwargs = dict(zip(keys, values))
        return _KwArgs(**kwargs)

    def _make_slice(expr):
        """Parse a slice object from a string. """
        def to_piece(s):
            return s and int(s) or None
        pieces = map(to_piece, expr.split(':'))
        if len(pieces) == 1:
            return slice(pieces[0], pieces[0] + 1)
        else:
            return slice(*pieces)

    def make_slice_arg(*args):
        """Create a scalar or tuple for acessing objects via slices. """
        out = [None] * len(args)
        for i, arg in enumerate(args):
            if isinstance(arg, str):
                out[i] = _make_slice(arg)
            else:
                out[i] = arg
        # Special case: If single index, collapse.
        if len(out) == 1:
            return out[0]
        else:
            return tuple(out)

    def setvar(var, value):
        """Sets a variable in the client's locals. """
        scope_locals[var] = value

    def setvars(*args):
        """Sets multiple variables in the client's locals. """
        scope_locals.update(make_kwargs(*args))

    def print(x):
        pass

    _execution_in_progress = _ExecutionInProgress()
    start_execution = _execution_in_progress.push
    finish_execution = _execution_in_progress.pop

    out = locals().copy()
    # Scrub extra stuff.
    del out["scope_locals"]
    return out


class _KwArgs(dict):
    # Indicates values meant solely for `**kwargs`.
    pass


def _cexec(stmt, globals_, locals_):
    # Enable executing statement via evaluation.
    eval_locals = dict(stmt=stmt, locals_=locals_, _cexec_impl=_cexec_impl)
    # Dispatch to function that calls "exec" so that we can control locals
    # and globals.
    eval("_cexec_impl(stmt, locals_)", eval_locals, globals_)


def _cexec_impl(_stmt, _locals):
    # Record old things.
    locals().update(_locals)
    _old_vars = None
    _new_vars = None
    _old_vars = locals().keys()
    # Execute with context.
    exec _stmt
    # Figure out new things.
    _new_vars = set(locals().keys()) - set(_old_vars)
    for var in _new_vars:
        _locals[var] = locals()[var]


def _merge_dicts(*args):
    # Merges a list of dict's.
    out = {}
    for arg in args:
        out.update(arg)
    return out


def default_globals():
    """Creates default globals for code that the client side can execute.

    This is geared for convenient (not necessarily efficient) plotting
    with `matplotlib`.
    """
    # @note This imports modules at a function-scope rather than at a
    # module-scope, which does not satisfy PEP8. This is intentional, as it
    # allows for a cleaner scope separation between the client core code (e.g.
    # `CallPythonClient`) and the client user code (e.g. `plot(x, y)`).
    # TODO(eric.cousineau): Consider relegating this to a different module,
    # possibly when this falls under `pydrake`.
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib
    import matplotlib.pyplot as plt
    import pylab  # See `%pylab?` in IPython.

    # TODO(eric.cousineau): Where better to put this?
    matplotlib.interactive(True)

    def disp(value):
        """Alias for print."""
        print(value)

    def wait():
        """Waits to allow user interaction with plots."""
        plt.show(block=True)

    def pause(interval):
        """Pause for `interval` seconds, letting the GUI flush its event queue.

        @note This is a *necessary* function to be defined if these globals are
        not used!
        """
        plt.pause(interval)

    def surf(x, y, Z, rstride=1, cstride=1, **kwargs):
        """Plots a 3d surface."""
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        X, Y = np.meshgrid(x, y)
        ax.plot_surface(X, Y, Z, rstride=rstride, cstride=cstride, **kwargs)

    def show():
        """Shows `matplotlib` images without blocking.

        Generally not needed if `matplotlib.is_interactive()` is true.
        """
        plt.show(block=False)

    def magic(N):
        """Provides simple odd-only case for magic squares.

        @ref https://scipython.com/book/chapter-6-numpy/examples/creating-a-magic-square  # noqa
        """
        assert N % 2 == 1
        magic_square = np.zeros((N, N), dtype=int)
        n = 1
        i, j = 0, N//2
        while n <= N**2:
            magic_square[i, j] = n
            n += 1
            newi, newj = (i - 1) % N, (j + 1) % N
            if magic_square[newi, newj]:
                i += 1
            else:
                i, j = newi, newj
        return magic_square

    # Use <module>.__dict__ to simulate `from <module> import *`, since that is
    # normally invalid in a function with nested functions.
    return _merge_dicts(
        globals(),
        plt.__dict__,
        pylab.__dict__,
        locals())


_FILENAME_DEFAULT = "/tmp/python_rpc"


class CallPythonClient(object):
    """Provides a client to receive Python commands.

    Enables printing or plotting from a C++ application for debugging
    purposes.
    """
    def __init__(self, filename=None, stop_on_error=True,
                 scope_globals=None, scope_locals=None,
                 threaded=True, loop=False):
        if filename is None:
            self.filename = _FILENAME_DEFAULT
        else:
            self.filename = filename
        # Scope. Give it access to everything here.
        # However, keep it's written values scoped.
        if scope_locals is None:
            self.scope_locals = {}
        else:
            self.scope_locals = scope_locals
        # Define globals as (a) required helpers for C++ interface, and
        # (b) convenience plotting functionality.
        # N.B. The provided locals OR globals can shadow the helpers. BE
        # CAREFUL!
        required_helpers = _get_required_helpers(self.scope_locals)
        if scope_globals is None:
            scope_globals = default_globals()
        self.scope_globals = _merge_dicts(required_helpers, scope_globals)

        self._stop_on_error = stop_on_error
        self._threaded = threaded
        self._loop = loop

        # Variables indexed by GUID.
        self._client_vars = {}

        self._had_error = False
        self._done = False
        self._file = None

    def _to_array(self, arg, dtype):
        # Converts a protobuf argument to the appropriate NumPy array (or
        # scalar).
        np_raw = np.frombuffer(arg.data, dtype=dtype)
        if arg.shape_type == MatlabArray.SCALAR:
            assert arg.cols == 1 and arg.rows == 1
            return np_raw[0]
        elif arg.shape_type == MatlabArray.VECTOR:
            assert arg.cols == 1
            return np_raw.reshape(arg.rows)
        elif arg.shape_type is None or arg.shape_type == MatlabArray.MATRIX:
            # TODO(eric.cousineau): Figure out how to ensure `np.frombuffer`
            # creates a column-major array?
            return np_raw.reshape(arg.cols, arg.rows).T

    def _execute_message(self, msg):
        # Executes a message, handling / recording that an error occurred.
        if self._stop_on_error:
            # Do not wrap in a `try` / `catch` to simplify debugging.
            self._execute_message_impl(msg)
        else:
            try:
                self._execute_message_impl(msg)
            except Exception as e:
                sys.stderr.write("ERROR: {}\n".format(e))
                sys.stderr.write("  Continuing (no --stop_on_error)\n")
                self._had_error = True

    def _execute_message_impl(self, msg):
        # Executes relevant portions of a message.
        # Create input arguments.
        args = msg.rhs
        nargs = len(args)
        inputs = []
        kwargs = None
        for i, arg in enumerate(args):
            arg_raw = arg.data
            value = None
            if arg.type == MatlabArray.REMOTE_VARIABLE_REFERENCE:
                id = np.frombuffer(arg_raw, dtype=np.uint64).reshape(1)[0]
                if id not in self._client_vars:
                    raise RuntimeError("Unknown local variable. " +
                                       "Dropping message.")
                value = self._client_vars[id]
            elif arg.type == MatlabArray.DOUBLE:
                value = self._to_array(arg, np.double)
            elif arg.type == MatlabArray.CHAR:
                assert arg.rows == 1
                value = str(arg_raw)
            elif arg.type == MatlabArray.LOGICAL:
                value = self._to_array(arg, np.bool)
            elif arg.type == MatlabArray.INT:
                value = self._to_array(arg, np.int32)
            else:
                assert False
            if isinstance(value, _KwArgs):
                assert kwargs is None
                kwargs = value
            else:
                inputs.append(value)

        # Call the function
        # N.B. No security measures to sanitize function name.
        function_name = msg.function_name
        out_id = None
        if len(msg.lhs) > 0:
            assert len(msg.lhs) == 1
            out_id = msg.lhs[0]

        self.scope_locals.update(_tmp_args=inputs, _tmp_kwargs=kwargs or {})
        # N.B. No try-catch block here. Can change this if needed.
        if function_name == "exec":
            assert len(inputs) == 1
            assert kwargs is None or len(kwargs) == 0
            _cexec(inputs[0], self.scope_globals, self.scope_locals)
            out = None
        else:
            out = eval(function_name + "(*_tmp_args, **_tmp_kwargs)",
                       self.scope_globals, self.scope_locals)
        self.scope_locals.update(_tmp_out=out)
        # Update outputs.
        self._client_vars[out_id] = out

    def run(self):
        """Runs the client code.

        @return True if no error encountered.
        """
        if self._threaded:
            self._handle_messages_threaded()
        else:
            self.handle_messages(record=False)
        # Check any execution in progress.
        exec_count = self.scope_globals['_execution_in_progress'].count
        if not self._had_error and exec_count != 0:
            self._had_error = True
            sys.stderr.write(
                "ERROR: Invalid termination. " +
                "'finish_execution' called insufficient number of " +
                "times: {}\n".format(exec_count))
        return not self._had_error

    def _handle_messages_threaded(self):
        # Handles messages in a threaded fashion.
        # Start producer thread (reading messages from file).
        queue = Queue()

        def producer_loop():
            for msg in self._read_next_message():
                msg_copy = copy.deepcopy(msg)
                queue.put(msg_copy)
                # Check if an error occurred.
                if self._done:
                    break
            self._done = True

        producer = Thread(target=producer_loop)
        producer.start()

        # Consume.
        # TODO(eric.cousineau): Trying to quit via Ctrl+C is awkward (but kinda
        # works). Is there a way to have `plt.pause` handle Ctrl+C differently?
        try:
            pause = self.scope_globals['pause']
            while not self._done:
                # Process messages.
                while not queue.empty():
                    msg = queue.get()
                    self._execute_message(msg)
                    queue.task_done()
                # Spin busy for a bit, let matplotlib (or whatever) flush its
                # event queue.
                pause(0.001)
        except KeyboardInterrupt:
            # User pressed Ctrl+C.
            self._done = True
            print("Quitting")
        except Exception as e:
            # We encountered an error, and must stop.
            self._done = True
            self._had_error = True
            sys.stderr.write("ERROR: {}\n".format(e))
            sys.stderr.write("  Stopping (--stop_on_error)\n")
        producer.join()

    def handle_messages(self, max_count=None, record=True, execute=True):
        """Handle all messages sent (e.g., through IPython).
        @param max_count Maximum number of messages to handle.
        @param record Record all messages and return them.
        @param execute Execute the given message upon receiving it.
        @return (count, msgs) where `count` is how many messages were processed
        (e.g. 0 if no more messages left).
        and `msgs` are either the messages themselves for playback.
        and (b) the messages themselves for playback (if record==True),
        otherwise an empty list.
        """
        assert record or execute, "Not doing anything useful?"
        count = 0
        msgs = []
        for msg in self._read_next_message():
            if execute:
                self._execute_message(msg)
            count += 1
            if record:
                msgs.append(copy.deepcopy(msg))
            if max_count is not None and count >= max_count:
                break
        return (count, msgs)

    def execute_messages(self, msgs):
        """Executes a set of recorded messages."""
        for msg in msgs:
            self._execute_message(msg)

    def _read_next_message(self):
        # Returns a new incoming message using a generator.
        # Not guaranteed to be a unique instance. Should copy if needed.
        msg = MatlabRPC()
        while not self._done:
            f = self._get_file()
            while not self._done:
                status = _read_next(f, msg)
                if status == _READ_GOOD:
                    yield msg
                elif status == _READ_NOT_READY:
                    pass
                elif status == _READ_END:
                    break
            # Close the file if we reach the end, NOT when exiting the scope
            # (which is why `with` is not used here).
            # This way the user can read a few messages at a time, with the
            # same file handle.
            # @note We must close / reopen the file when looping because the
            # C++ program will effectively send a EOF signal when it closes
            # the pipe.
            self._close_file()
            if not self._loop:
                break

    def _get_file(self):
        # Gets file handle, opening if needed.
        if self._file is None:
            # @ref https://stackoverflow.com/q/27822560/7829525#comment44053169_27822560  # noqa
            fd = os.open(self.filename, os.O_RDONLY | os.O_NONBLOCK)
            self._file = os.fdopen(fd)
        return self._file

    def _close_file(self):
        # Closes file if open.
        if self._file is not None:
            self._file.close()
            self._file = None


_READ_GOOD = 1
_READ_NOT_READY = 2
_READ_END = 3


def _read_next(f, msg):
    # Reads next message from the given file, following suite with C++.
    # Use "select" so that we do not get in odd deadlocks with FIFO pipes.
    # timeout = 0.005
    read_streams = [f]
    read_streams_ready = [f]  #select.select(read_streams, [], [], timeout)[0]
    if f not in read_streams_ready:
        # Stream is not ready. Busy-spin.
        return _READ_NOT_READY
    # Number of bytes we need to consume so that we may still use
    # `_DecodeVarint32`.
    header_size = 4
    msg_size_raw = f.read(header_size)
    if len(msg_size_raw) == 0:
        # We have reached the end.
        return _READ_END
    msg_size = np.frombuffer(msg_size_raw, dtype=np.int32)[0]
    msg_raw = f.read(msg_size)
    assert len(msg_raw) == msg_size
    # Now read the message.
    msg.ParseFromString(msg_raw)
    return _READ_GOOD


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no_loop", action='store_true',
        help="Do not loop the end for user interaction. With a FIFO pipe, " +
             "this will end session after C++ program closes.")
    parser.add_argument(
        "--no_threading", action='store_true',
        help="Disable threaded dispatch.")
    parser.add_argument("--stop_on_error", action='store_true',
                        help="Stop client if there is an error when " +
                             "executing a call.")
    parser.add_argument("-f", "--file", type=str, default=None)
    args = parser.parse_args(argv)

    client = CallPythonClient(
        args.file, stop_on_error=args.stop_on_error,
        threaded=not args.no_threading, loop=not args.no_loop)
    good = client.run()
    return good


if __name__ == "__main__":
    import cProfile, pstats
    pr = cProfile.Profile()
    pr.enable()
    good = main(sys.argv[1:])
    pr.disable()
    pr.print_stats(1)
    if not good:
        exit(1)
