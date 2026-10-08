# pyright's stub for mujoco: its bindings are compiled (pybind11) and ship no stubs, so
# pyright knew none of its attributes. The module __getattr__ makes them Any (see
# typings/OCP/__init__.pyi).
from typing import Any

def __getattr__(name: str) -> Any: ...
