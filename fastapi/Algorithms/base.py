
from __future__ import annotations
from typing import Any, Dict, List, Tuple

class BaseAlgorithm:

    name = "BaseAlgorithm"

    def solve(self,
              graph,
              start: str,
              goal: str,
              vehicle,
              weights: Dict[str, float],
              positions: Dict[str, tuple] | None = None) -> Dict[str, Any]:
        raise NotImplementedError("Implement in subclass")
