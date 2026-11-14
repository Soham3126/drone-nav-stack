# Make `import drone_bridge` find this package's sources whatever directory pytest is started from.
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
