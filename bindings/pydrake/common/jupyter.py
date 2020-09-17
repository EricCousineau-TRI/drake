"""
Provides support for using ipywidgets with the systems framework, and a number
of useful widget systems.

This is gui code; to test changes, please manually run
//bindings/pydrake/systems/jupyter_widgets_examples.ipynb.
"""

import asyncio
from contextlib import contextmanager
import functools
from packaging import version
import sys
from warnings import warn

from IPython import get_ipython
from IPython.display import clear_output, display
import ipywidgets as widgets

# Note: The implementation below was inspired by
# https://github.com/Kirill888/jupyter-ui-poll , though I suspect it can be
# optimized.
#
# For reference,
# https://ipywidgets.readthedocs.io/en/latest/examples/Widget%20Asynchronous.html  # noqa
# describes the problem but it only offers the solution of using separate
# threads for execution; a workflow that we do not wish to impose on users.

def process_ipywidget_events(num_events_to_process=1):
    """
    Allows the kernel to process GUI events.  This is required in order to
    process ipywidget updates inside a simulation loop.
    """

    shell = get_ipython()
    # Ok to do nothing if running from console.
    if shell is None or not hasattr(shell, 'kernel'):
        return
    kernel = shell.kernel
    events = []
    old_handler = kernel.shell_handlers['execute_request']
    kernel.shell_handlers['execute_request'] = lambda *e: events.append(e)
    current_parent = (kernel._parent_ident, kernel._parent_header)

    for _ in range(num_events_to_process):
        # Ensure stdout still happens in the same cell.
        kernel.set_parent(*current_parent)
        kernel.do_one_iteration()
        kernel.set_parent(*current_parent)

    kernel.shell_handlers['execute_request'] = old_handler

    def _replay_events(shell, events):
        kernel = shell.kernel
        sys.stdout.flush()
        sys.stderr.flush()
        for stream, ident, parent in events:
            kernel.set_parent(ident, parent)
            if kernel._aborting:
                kernel._send_abort_reply(stream, parent, ident)
            else:
                kernel.execute_request(stream, ident, parent)

    if len(events) > 0:
        loop = asyncio.get_event_loop()
        if loop.is_running():
            loop.call_soon(lambda: _replay_events(shell, events))
        else:
            warn("One of your components is attempting to use pydrake's "
                 "process_ipywidget_events function. However, this IPython "
                 "kernel is not asyncio-based. This means the following:\n"
                 "  (1) Once your block cell is done executing, future cells "
                 "will *not* execute, but it may appear like they are still "
                 "executing ([*]).\n"
                 "  (2) Your Jupyter UI may break. If you find your UI to be "
                 "unresponsive, you may need to restart the UI itself.\n"
                 "To avoid this behavior, avoid requesting execution of "
                 "future cells before or during execution of this cell.")


def _maybe_show_inline_matplotlib_plots():
    if version.parse(widgets.__version__) >= version.parse("7.0.0"):
        widgets.widgets.interaction.show_inline_matplotlib_plots()


@contextmanager
def interactive_update(out):
    """
    Provides a context that will clear an Output widget beforehand, focus on it,
    and then ensure that matplotlib plots are updated appropriately (for
    ipywidgets>=7.0.0).

    Arguments:
        out (Output): The Output widget to which output should be directed.

    Note:
        This is a more generalized form of ``interactive_output``:
        https://github.com/jupyter-widgets/ipywidgets/blob/7.5.1/ipywidgets/widgets/interaction.py#L65-L85
    """  # noqa
    assert isinstance(out, widgets.Output), out
    with out:
        clear_output(wait=True)
        yield
        _maybe_show_inline_matplotlib_plots()


def decorate_interactive_update(out):
    """
    Decorates a function to be executed within a `interactive_update` context.

    Arguments:
        out (Output): The Output widget to which output should be directed.
    """
    assert isinstance(out, widgets.Output), out

    def decorator(f):

        @functools.wraps(f)
        def wrapped(*args, **kwargs):
            with interactive_update(out):
                return f(*args, **kwargs)

        return wrapped

    return decorator
