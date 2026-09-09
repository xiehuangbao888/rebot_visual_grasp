"""Load Ultralytics YOLO / YOLOE models for visual grasp."""

from __future__ import annotations

import importlib
import site
import sys
from pathlib import Path
from typing import Any, Optional

_PREPARED = False


def _is_yoloe_model(model_path: str) -> bool:
    name = Path(model_path).name.lower()
    return "yoloe" in name or "yolo-world" in name or "world" in name


def _ultralytics_search_paths() -> list[str]:
    return [path for path in site.getsitepackages() + [site.getusersitepackages()] if path]


def _prepare_ultralytics_import() -> None:
    """Use pip-installed ultralytics instead of a shadowed ./ultralytics source checkout."""
    global _PREPARED
    if _PREPARED:
        return

    site_paths = _ultralytics_search_paths()
    cwd = Path.cwd().resolve()

    for name in list(sys.modules):
        if name != "ultralytics" and not name.startswith("ultralytics."):
            continue
        mod_file = getattr(sys.modules[name], "__file__", "") or ""
        if "site-packages" not in mod_file.replace("\\", "/"):
            del sys.modules[name]

    filtered_path: list[str] = []
    for entry in sys.path:
        if entry == "":
            continue
        try:
            if Path(entry).resolve() == cwd:
                continue
        except OSError:
            pass
        filtered_path.append(entry)

    for path in reversed(site_paths):
        if path not in filtered_path:
            filtered_path.insert(0, path)

    sys.path[:] = filtered_path
    _PREPARED = True


def _import_ultralytics_symbol(symbol: str) -> Any:
    _prepare_ultralytics_import()
    module = importlib.import_module("ultralytics")
    return getattr(module, symbol)


def resolve_model_weights(model_path: str) -> str:
    """Resolve a local .pt path, downloading Ultralytics hub weights when needed."""
    raw = str(model_path).strip()
    if not raw:
        raise ValueError("yolo_model is empty")

    candidate = Path(raw).expanduser()
    if candidate.is_file():
        return str(candidate.resolve())

    _prepare_ultralytics_import()
    settings_mod = importlib.import_module("ultralytics.utils")
    downloads_mod = importlib.import_module("ultralytics.utils.downloads")
    SETTINGS = settings_mod.SETTINGS
    attempt_download_asset = downloads_mod.attempt_download_asset

    weights_dir = Path(str(SETTINGS["weights_dir"])).expanduser()
    if not weights_dir.is_absolute():
        weights_dir = Path.cwd() / weights_dir
    in_weights = weights_dir / candidate.name
    if in_weights.is_file():
        return str(in_weights.resolve())

    downloaded = attempt_download_asset(candidate.name)
    downloaded_path = Path(downloaded).expanduser()
    if downloaded_path.is_file():
        return str(downloaded_path.resolve())
    if (weights_dir / downloaded_path.name).is_file():
        return str((weights_dir / downloaded_path.name).resolve())

    raise FileNotFoundError(
        f"YOLO weights not found: {raw}. Download manually, e.g.\n"
        f"  wget https://github.com/ultralytics/assets/releases/download/v8.4.0/{candidate.name}"
    )


def _stage_yoloe_text_assets(model_path: str) -> None:
    """Put MobileCLIP TorchScript where Ultralytics attempt_download_asset can find it."""
    asset_name = "mobileclip2_b.ts"
    search_roots = [
        Path(model_path).resolve().parent,
        Path("/home/ubuntu/ultralytics-main"),
        Path.home() / "ultralytics-main",
        Path.cwd(),
        Path.cwd() / "weights",
    ]
    local = next((root / asset_name for root in search_roots if (root / asset_name).is_file()), None)
    if local is None:
        return

    # Prefer Ultralytics weights_dir, fall back to CWD.
    try:
        _prepare_ultralytics_import()
        settings_mod = importlib.import_module("ultralytics.utils")
        weights_dir = Path(str(settings_mod.SETTINGS["weights_dir"])).expanduser()
        if not weights_dir.is_absolute():
            weights_dir = Path.cwd() / weights_dir
    except Exception:
        weights_dir = Path.cwd() / "weights"

    targets = [Path.cwd() / asset_name, weights_dir / asset_name]
    for target in targets:
        if target.resolve() == local.resolve():
            continue
        if target.is_file() and target.stat().st_size >= local.stat().st_size:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        try:
            if target.exists() or target.is_symlink():
                target.unlink()
            target.symlink_to(local)
        except OSError:
            import shutil

            shutil.copy2(local, target)


def load_yolo_model(
    model_path: str,
    *,
    custom_classes: Optional[list[str]] = None,
    use_yoloe: Optional[bool] = None,
) -> Any:
    """Load YOLO or YOLOE. YOLOE models call set_classes() when custom_classes is set."""
    path = resolve_model_weights(model_path)
    yoloe = _is_yoloe_model(path) if use_yoloe is None else bool(use_yoloe)
    classes = [str(name).strip() for name in (custom_classes or []) if str(name).strip()]

    if yoloe:
        try:
            YOLOE = _import_ultralytics_symbol("YOLOE")
        except (ImportError, AttributeError) as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError(
                "YOLOE requires ultralytics>=8.4 with YOLOESegment26 support. "
                "Run: pip install -U ultralytics"
            ) from exc
        _stage_yoloe_text_assets(path)
        model = YOLOE(path)
        if classes:
            try:
                model.set_classes(classes)
            except ModuleNotFoundError as exc:
                if "clip" in str(exc).lower():
                    raise RuntimeError(
                        "YOLOE custom classes need Ultralytics CLIP tokenizer. Install with:\n"
                        "  pip uninstall -y clip\n"
                        "  pip install git+https://github.com/ultralytics/CLIP.git\n"
                        "Do NOT use the PyPI package named 'clip'."
                    ) from exc
                raise
            except (ConnectionError, FileNotFoundError, OSError) as exc:
                raise RuntimeError(
                    "YOLOE set_classes needs MobileCLIP weights (~242MB):\n"
                    "  cd ~/ultralytics-main\n"
                    "  wget -c https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts\n"
                    f"Current error: {exc}"
                ) from exc
        return model

    YOLO = _import_ultralytics_symbol("YOLO")
    return YOLO(path)


def normalize_yolo_device(device: str) -> str:
    """Map friendly aliases to Ultralytics device ids.

    Ultralytics rejects bare 'gpu'; use 'cpu' or '0'/'0,1',...
    """
    value = str(device).strip().lower()
    if not value:
        return "cpu"
    if value in {"gpu", "cuda", "cuda:0"}:
        return "0"
    if value.startswith("cuda:"):
        # cuda:1 -> 1
        return value.split(":", 1)[1] or "0"
    return str(device).strip()


def predict_kwargs(device: str, conf_threshold: float) -> dict[str, Any]:
    kwargs: dict[str, Any] = {"verbose": False, "conf": conf_threshold}
    device = normalize_yolo_device(device)
    # If caller asked for GPU but CUDA is down, fall back to CPU instead of crashing.
    if device not in {"", "cpu"}:
        try:
            import torch

            if not torch.cuda.is_available():
                device = "cpu"
        except Exception:
            device = "cpu"
    if device:
        kwargs["device"] = device
    return kwargs


def ultralytics_available() -> bool:
    try:
        _import_ultralytics_symbol("YOLO")
        return True
    except Exception:
        return False
