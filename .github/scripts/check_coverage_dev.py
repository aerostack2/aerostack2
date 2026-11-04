"""Check that the coverage environment lists every aerostack2 package.

The `[feature.coverage.dev]` table in pixi.toml must name the same packages,
with the same paths, as `[package.run-dependencies]` in aerostack2/pixi.toml.
"""

from pathlib import Path
import re
import sys
import tomllib

ROOT = Path(__file__).resolve().parents[2]


def load_table(path, table):
    # Parse only this table: other parts of the manifests use TOML 1.1 syntax
    # (multi-line inline tables) that pixi accepts but tomllib does not.
    text = (ROOT / path).read_text()
    match = re.search(rf'^\[{re.escape(table)}\]\n(.*?)(?=^\[|\Z)', text, re.M | re.S)
    if not match:
        sys.exit(f'{path}: no [{table}] table')
    return tomllib.loads(match.group(1))


def main():
    meta = load_table('aerostack2/pixi.toml', 'package.run-dependencies')
    dev = load_table('pixi.toml', 'feature.coverage.dev')

    # Paths are relative to each manifest: '../as2_core' vs './as2_core'.
    def rel(base, path):
        return (base / path).resolve().relative_to(ROOT)

    expected = {name: rel(ROOT / 'aerostack2', d['path']) for name, d in meta.items()}
    actual = {name: rel(ROOT, d['path']) for name, d in dev.items()}

    errors = []
    for name in sorted(expected.keys() - actual.keys()):
        errors.append(f'missing: {name} = {{ path = "./{expected[name]}" }}')
    for name in sorted(actual.keys() - expected.keys()):
        errors.append(f'not in aerostack2/pixi.toml: {name}')
    for name in sorted(expected.keys() & actual.keys()):
        if expected[name] != actual[name]:
            errors.append(f'wrong path: {name} is at ./{expected[name]}, not ./{actual[name]}')

    if errors:
        print('[feature.coverage.dev] in pixi.toml is out of sync with aerostack2/pixi.toml:')
        for e in errors:
            print(f'  {e}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
