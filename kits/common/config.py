"""Command-line config handling shared by the kit demos.

Each system a demo uses (a base, an arm, a camera mast, ...) gets its own
config file argument, so configs stay single-purpose and the command line
shows what's being combined. An argument is a path, or a file name found in
one of the demo's config directories (searched in order).
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import TYPE_CHECKING

import hebi

if TYPE_CHECKING:
    from collections.abc import Sequence


def _available(config_dirs: Sequence[str | Path]) -> str:
    names = sorted({p.name for d in config_dirs for p in Path(d).glob("*.cfg.yaml")})
    return ", ".join(names)


def config_arg_parser(description: str, *config_dirs: str | Path) -> argparse.ArgumentParser:
    """An ``ArgumentParser`` whose help lists the configs in ``config_dirs``. Add config arguments with ``add_config_argument``."""
    dirs = ", ".join(str(d) for d in config_dirs)
    return argparse.ArgumentParser(
        description=description,
        epilog=f"Config arguments are paths, or names in: {dirs}. Available: {_available(config_dirs)}",
    )


def add_config_argument(parser: argparse.ArgumentParser, name: str, help: str, *, option: bool = False, nargs: int | None = None):
    """Add a config file argument: positional ``name``, or ``--name`` if ``option`` (then it may be omitted)."""
    if option:
        parser.add_argument(f"--{name.replace('_', '-')}", dest=name, metavar=name.upper(), nargs=nargs, help=help)
    else:
        parser.add_argument(name, metavar=name.upper(), nargs=nargs, help=help)


def load_config(name: str | Path, *config_dirs: str | Path) -> hebi.config.HebiConfig:
    """Load a config by path, or by file name from the first of ``config_dirs`` that has it."""
    path = Path(name)
    if not path.exists():
        for config_dir in config_dirs:
            if (candidate := Path(config_dir) / name).exists():
                path = candidate
                break
        else:
            raise FileNotFoundError(f"No config file '{name}' (looked in {', '.join(str(d) for d in config_dirs)})")
    return hebi.config.load_config(str(path))


def user_data(cfg: hebi.config.HebiConfig, key: str, default=None):
    """``cfg.user_data[key]``, or ``default``."""
    return (cfg.user_data or {}).get(key, default)


def required_user_data(cfg: hebi.config.HebiConfig, key: str):
    """``cfg.user_data[key]``; raises ``KeyError`` naming the config if it's missing."""
    if (value := user_data(cfg, key)) is None:
        raise KeyError(f"Config for {cfg.families} {cfg.names} needs user_data '{key}'")
    return value
