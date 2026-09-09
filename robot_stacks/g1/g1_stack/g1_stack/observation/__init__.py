"""One frame of camera plus joints, assembled and validated."""

from .assembler import ObservationAssembler
from .types import Observation

__all__ = ["Observation", "ObservationAssembler"]
