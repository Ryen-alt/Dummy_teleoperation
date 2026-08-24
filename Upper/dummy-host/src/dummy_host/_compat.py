from __future__ import annotations

try:
    from enum import StrEnum
except ImportError:  # Python 3.10; enum.StrEnum was added in Python 3.11.
    from enum import Enum

    class StrEnum(str, Enum):
        def __str__(self) -> str:
            return str(self.value)


__all__ = ["StrEnum"]
