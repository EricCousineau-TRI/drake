from contextlib import contextmanager
from functools import wraps

import IPython.display
import ipywidgets.widgets.interaction
import ipywidgets as widgets


@contextmanager
def interactive_context(out):
    """
    Focuses on an output widget, first clearing its content, then showing
    any new plots.

    Adapted from `ipywidets.widgets.interaction`
    """
    with out:
        IPython.display.clear_output(wait=True)
        yield
        ipywidgets.widgets.interaction.show_inline_matplotlib_plots()


def interactive_decorator(out):
    """Decorator sugar for `interactive_context`."""

    def decorator(f):

        @wraps(f)
        def wrapped(*args, **kwargs):
            with interactive_context(out):
                return f(*args, **kwargs)

        return wrapped

    return decorator
