#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""Fail if a translation unit was compiled with NDEBUG or EIGEN_NO_DEBUG in effect.

"In effect" means EIGEN_NO_DEBUG is defined and not cancelled by a later -U. The
sanitizer job exists to run with Eigen's own bounds assertions live, which only
holds if the Debug build type keeps NDEBUG unset and the -UEIGEN_NO_DEBUG
carried in CMAKE_CXX_FLAGS actually cancels the -DEIGEN_NO_DEBUG every package
adds at directory scope. This reads the outcome back from compile_commands.json
instead of trusting the cmake-args alone, so the job cannot silently degrade
into a decoration if a future change reorders or drops one of the flags.

Usage: check_no_eigen_debug.py <compile_commands.json glob> [<glob> ...]
"""

import json
import pathlib
import shlex
import sys


def command_args(entry):
    """The argument list of one compile_commands.json entry."""
    if 'arguments' in entry:
        return entry['arguments']
    return shlex.split(entry.get('command', ''))


def problems_for(args):
    """Names for one translation unit's compile flags that violate the contract."""
    found = []
    if '-DNDEBUG' in args:
        found.append('NDEBUG is defined')
    eigen_flags = [a for a in args if a in ('-DEIGEN_NO_DEBUG', '-UEIGEN_NO_DEBUG')]
    if eigen_flags and eigen_flags[-1] != '-UEIGEN_NO_DEBUG':
        found.append('EIGEN_NO_DEBUG is defined and not cancelled by a later -U')
    return found


def main(argv):
    """Check every compile_commands.json entry matched by the given globs."""
    if not argv:
        print('usage: check_no_eigen_debug.py <compile_commands.json glob> [...]', file=sys.stderr)
        return 2

    failures = []
    checked = 0
    for pattern in argv:
        for path in sorted(pathlib.Path().glob(pattern)):
            entries = json.loads(path.read_text())
            for entry in entries:
                problems = problems_for(command_args(entry))
                checked += 1
                if problems:
                    failures.append((entry.get('file', str(path)), problems))

    if checked == 0:
        print('No compile_commands.json entries found; nothing was checked.', file=sys.stderr)
        return 2

    if failures:
        for file_path, problems in failures:
            for problem in problems:
                print(f'FAIL: {file_path}: {problem}', file=sys.stderr)
        return 1

    print(
        f'OK: {checked} translation units compiled with NDEBUG unset and '
        'EIGEN_NO_DEBUG cancelled by -U.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
