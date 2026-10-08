# pyright's stubs for OCP (cadquery-ocp): the package is compiled (pybind11) and ships no
# stubs, so pyright saw none of its names ("unknown import symbol"). Each submodule the code
# imports gets a stub whose module __getattr__ makes its names Any: no checking of OCP's own
# API, but no noise either, and nothing else in the importing files loses its checks. A new
# `from OCP.X import ...` needs typings/OCP/X.pyi (the same three lines).
from typing import Any

def __getattr__(name: str) -> Any: ...
