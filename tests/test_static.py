"""Source-level checks that need no hardware."""

import ast
import importlib.util
import io
import re
import tokenize

import hebi
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
    offenders = [f"{p.relative_to(REPO_ROOT)}" for p in sorted((KITS / "tready").glob("*.py")) if re.search(r"%H:%M", p.read_text())]
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


def _nested_fstring_quotes(source: str) -> list[int]:
    """Lines where a string inside an f-string reuses the f-string's quote (PEP 701, Python 3.12+)."""
    if not hasattr(tokenize, "FSTRING_START"):  # running on < 3.12: compile() already rejects them
        return []
    quotes, lines = [], []
    for tok in tokenize.generate_tokens(io.StringIO(source).readline):
        if tok.type in (tokenize.STRING, tokenize.FSTRING_START):
            quote = tok.string.lstrip("rRbBuUfF")[:3]
            quote = quote if quote in ('"""', "'''") else quote[0]
            if quotes and quotes[-1] == quote:
                lines.append(tok.start[0])
            if tok.type == tokenize.FSTRING_START:
                quotes.append(quote)
        elif tok.type == tokenize.FSTRING_END:
            quotes.pop()
    return lines


def test_kits_parse_on_python_3_11():
    """The demos support Python 3.11+: no PEP 695 generics/type aliases, no PEP 701 f-string quote reuse."""
    offenders = []
    for path in sorted(KITS.rglob("*.py")):
        source = path.read_text()
        rel = path.relative_to(REPO_ROOT)
        try:
            ast.parse(source, filename=str(path), feature_version=(3, 11))
        except SyntaxError as e:
            offenders.append(f"{rel}:{e.lineno}: {e.msg}")
        offenders += [f"{rel}:{line}: f-string reuses its quote" for line in _nested_fstring_quotes(source)]
    assert not offenders, "\n".join(offenders)


def test_ar_arm_demo_works_with_older_scipy():
    """A15: Rotation.from_quat(scalar_first=...) needs scipy >= 1.14 and raises TypeError before that."""
    for path in ["tready/treaded_base_arm_io_control.py", "camera/pan_tilt_mast.py", "arms/ar_control_sm.py"]:
        tree = ast.parse((KITS / path).read_text())
        assert not any(isinstance(n, ast.keyword) and n.arg == "scalar_first" for n in ast.walk(tree)), path


def test_treadward_control_selector_targets_exist():
    """With no payload found, the selector launched kits.tready.smart_treadward_control, which doesn't exist."""
    tree = ast.parse((KITS / "tready" / "treadward_control_selector.py").read_text())
    modules = [
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and re.fullmatch(r"kits(\.\w+)+", node.value)
    ]
    assert modules and all(importlib.util.find_spec(m) is not None for m in modules), modules


def test_only_the_controller_transitions_states():
    """States and demos never call the Controller's _transition_to: update() returns the next state, and the
    Controller applies it (kits/common/state_machine.py). receive_inputs() records requests; it never transitions."""
    offenders = []
    for path in sorted(KITS.rglob("*.py")):
        if path == KITS / "common" / "state_machine.py":
            continue
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr in ("transition_to", "_transition_to")
            ):
                offenders.append(f"{path.relative_to(REPO_ROOT)}:{node.lineno}")
    assert not offenders, f"_transition_to called outside the Controller: {offenders}"
