"""Compatibility import for the native Evo line-tracing controller."""

from _evo import EvoLineTrace

LEFT = EvoLineTrace.LEFT
RIGHT = EvoLineTrace.RIGHT
BOTH = EvoLineTrace.BOTH

__all__ = ("EvoLineTrace", "LEFT", "RIGHT", "BOTH")
