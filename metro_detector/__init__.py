"""Offline single-cloud metro obstacle detection."""
import os

# Avoid BLAS oversubscription for small per-frame matrix operations.
# Explicit deployment overrides are respected.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
__version__ = '0.4.2'
