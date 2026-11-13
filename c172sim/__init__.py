"""Educational reduced Cessna 172P simulation."""
from .aircraft import Aircraft, Controls
from .dynamics import rhs
from .trim import level_trim
from .simulation import simulate_segments
__version__ = "0.5.0"
