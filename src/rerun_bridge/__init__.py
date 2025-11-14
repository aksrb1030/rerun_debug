"""Utility package for the ROS 2 to Rerun bridge."""

__all__ = ["main"]


def __getattr__(name):
    """Lazy import to avoid conflicts when running as __main__."""
    if name == "main":
        from .main import main
        return main
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
