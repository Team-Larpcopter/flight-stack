#!/usr/bin/env python3
"""Run in the known-good Jetson LAS container; export runtime only, never compile."""
import glob
import importlib
import json
from pathlib import Path
import sys
import tarfile

output = sys.argv[1]
modules = ['numpy', 'torch', 'torchgen', 'tensorrt', 'filelock', 'fsspec', 'jinja2', 'networkx',
           'sympy', 'typing_extensions', 'mpmath', 'markupsafe']
paths = []
versions = {}
for name in modules:
    module = importlib.import_module(name)
    path = Path(module.__file__)
    paths.append(path.parent if path.name == '__init__.py' else path)
    versions[name] = getattr(module, '__version__', 'unknown')
    if name == 'numpy' and (path.parent.parent / 'numpy.libs').is_dir():
        paths.append(path.parent.parent / 'numpy.libs')
cuda = Path('/usr/local/cuda').resolve()
for pattern in [str(cuda / 'targets/aarch64-linux/lib/*.so*'),
                '/usr/lib/aarch64-linux-gnu/libnvinfer*.so*',
                '/usr/lib/aarch64-linux-gnu/libnvonnxparser*.so*',
                '/usr/lib/aarch64-linux-gnu/libcudnn*.so*',
                '/usr/lib/aarch64-linux-gnu/libcusparseLt.so*',
                '/usr/share/doc/libnvinfer*/copyright',
                str(cuda / 'EULA.txt')]:
    paths.extend(Path(p) for p in glob.glob(pattern))
# Drivers belong to the Jetson host and are injected by NVIDIA Container Runtime.
# Keep original absolute paths; preserve shared-library symlinks.
with tarfile.open(output, 'w:gz', compresslevel=1) as tar:
    for path in sorted(set(paths)):
        tar.add(path, arcname=str(path).lstrip('/'),
                filter=lambda item: None if '__pycache__' in item.name else item)
Path(output + '.json').write_text(json.dumps(versions, indent=2) + '\n')
