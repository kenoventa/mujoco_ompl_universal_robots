import sys
import os

# Get the absolute path to the current directory (subdir)
current_dir = os.path.dirname(__file__)

# Add the current directory to sys.path (if it's not already there)
if current_dir not in sys.path:
    sys.path.append(current_dir)