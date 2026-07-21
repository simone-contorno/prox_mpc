#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Fail if a package.xml declares a dependency the license inventory does not cover.

Two dependencies (``ffmpeg``, ``nav2_graceful_controller``) reached the tree
without an inventory row because nothing gated it. This closes that gap: every
rosdep key must be named either in the THIRD_PARTY_LICENSES.md inventory table
or in its enumerated ROS 2 distribution coverage list.

Intra-repository dependencies (the prox_mpc_* packages) are covered by the
repository LICENSE and are skipped.
"""

import pathlib
import re
import sys
import xml.etree.ElementTree as ET

DEPEND_TAGS = (
    'depend',
    'build_depend',
    'build_export_depend',
    'buildtool_depend',
    'exec_depend',
    'test_depend',
)

# Aliases: the inventory names a library by its project name, package.xml by its
# rosdep key.
ALIASES = {
    'eigen': 'eigen3',
    'proxsuite': 'proxsuite',
}


def declared_keys(root):
    """Every rosdep key declared by any package.xml under root, minus our own."""
    keys = set()
    own = {p.parent.name for p in root.glob('*/package.xml')}
    for manifest in sorted(root.glob('*/package.xml')):
        tree = ET.parse(manifest)
        for tag in DEPEND_TAGS:
            for node in tree.getroot().iter(tag):
                if node.text:
                    keys.add(node.text.strip())
    return keys - own


def inventoried_names(inventory):
    """Names the inventory mentions: backticked tokens plus the coverage list."""
    text = inventory.read_text()
    names = {t.lower() for t in re.findall(r'`([A-Za-z0-9_.+-]+)`', text)}
    for block in re.findall(r'```text\n(.*?)```', text, re.DOTALL):
        # Only the coverage list is a bare key-per-line block; license texts are
        # prose and contribute nothing that looks like a key.
        for line in block.splitlines():
            token = line.strip()
            if token and re.fullmatch(r'[A-Za-z0-9_.+-]+', token):
                names.add(token.lower())
    return names


def main():
    root = pathlib.Path(__file__).resolve().parents[2]
    inventory = root / 'THIRD_PARTY_LICENSES.md'
    if not inventory.is_file():
        print(f'error: {inventory} not found', file=sys.stderr)
        return 2

    covered = inventoried_names(inventory)
    missing = sorted(
        key for key in declared_keys(root)
        if key.lower() not in covered
        and ALIASES.get(key.lower(), key.lower()) not in covered
        and f'ros-jazzy-{key.lower().replace("_", "-")}' not in covered
    )

    if missing:
        print('Dependencies declared in package.xml but absent from', file=sys.stderr)
        print(f'{inventory.name} (add an inventory row or a coverage entry):', file=sys.stderr)
        for key in missing:
            print(f'  - {key}', file=sys.stderr)
        return 1

    print(f'OK: every declared dependency is covered by {inventory.name}.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
