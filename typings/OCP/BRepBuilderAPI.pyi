# OCP is compiled (pybind11) with no stubs: every name it exports is Any (typings/OCP/__init__.pyi)
from typing import Any

def __getattr__(name: str) -> Any: ...
