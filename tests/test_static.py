"""Source-level checks that need no hardware."""

import ast
import importlib.util
import re

import hebi
import pytest

from conftest import REPO_ROOT

KITS = REPO_ROOT / "kits"


def test_add_waypoint_always_has_t():
    """A1: `t` is a required argument of Goal.add_waypoint in hebi-py >= 2.16.1."""
    missing = []
    for path in sorted(KITS.rglob("*.py")):
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "add_waypoint"
                and not node.args
                and not any(kw.arg == "t" for kw in node.keywords)
            ):
                missing.append(f"{path.relative_to(REPO_ROOT)}:{node.lineno}")
    assert not missing, f"add_waypoint without t: {missing}"


def test_double_shoulder_mirror_group_name():
    """A6: `group_name: [J2B_shoulder1"]` parsed as the name 'J2B_shoulder1"' (stray quote)."""
    bad = {}
    for path in sorted((KITS / "arms" / "config").glob("*.cfg.yaml")):
        cfg = hebi.config.load_config(str(path))
        for plugin in cfg.plugins or []:
            if plugin.get("type") == "DoubledJointMirror" and plugin.get("group_name") != ["J2B_shoulder1"]:
                bad[path.name] = plugin.get("group_name")
    assert not bad


def test_log_names_have_no_colons():
    """A17: ':' is not allowed in Windows file names."""
    offenders = [
        f"{p.relative_to(REPO_ROOT)}"
        for p in sorted((KITS / "tready").glob("*.py"))
        if re.search(r"%H:%M", p.read_text())
    ]
    assert not offenders


def test_arm_demos_parse_before_python_3_12():
    """A13: an f-string expression containing a backslash ({'\\n'.join(...)}) is a SyntaxError before Python 3.12."""
    pattern = re.compile(r"""f(["']).*\{[^}]*\\[^}]*\}.*\1""")
    offenders = [
        f"{p.relative_to(REPO_ROOT)}:{i}"
        for p in sorted(KITS.rglob("*.py"))
        for i, line in enumerate(p.read_text().splitlines(), 1)
        if pattern.search(line)
    ]
    assert not offenders


def test_ar_arm_demo_works_with_older_scipy():
    """A15: Rotation.from_quat(scalar_first=...) needs scipy >= 1.14 and raises TypeError before that."""
    assert "scalar_first" not in (KITS / "tready" / "tready_arm_io_control.py").read_text()


def test_demo_selector_imports_sys_and_shows_demo_output():
    """A7: demo_selector used sys without importing it, and check_output swallowed the demo's console output."""
    source = (KITS / "tready" / "demo_selector.py").read_text()
    tree = ast.parse(source)
    imported = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    assert "sys" in imported
    assert "check_output" not in source


def test_demo_selector_targets_exist():
    from kits.tready.demo_selector import DEMOS

    missing = [module for module in DEMOS.values() if importlib.util.find_spec(module) is None]
    assert not missing


def test_treadward_control_selector_fallback_exists():
    """A8: with no payload found, the selector launched kits.tready.smart_treadward_control, which doesn't exist."""
    tree = ast.parse((KITS / "tready" / "treadward_control_selector.py").read_text())
    modules = [
        node.value.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "default_controller" for t in node.targets)
    ]
    assert modules and all(importlib.util.find_spec(m) is not None for m in modules), modules
