"""Validate an explicitly selected map without changing any map files."""
import hashlib
import math
from pathlib import Path
import yaml


def selected_map(path, legacy_path, legacy_yaml_sha, legacy_image_sha):
    path = Path(path).expanduser().resolve(strict=True)
    raw = path.read_bytes()
    config = yaml.safe_load(raw)
    if not isinstance(config, dict) or not isinstance(config.get('image'), str):
        raise ValueError('Map YAML requires an image path')
    resolution = config.get('resolution')
    origin = config.get('origin')
    if (not isinstance(resolution, (int, float)) or not math.isfinite(resolution)
            or resolution <= 0 or not isinstance(origin, list) or len(origin) != 3
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in origin)):
        raise ValueError('Invalid map resolution or origin')
    image = (path.parent / config['image']).resolve(strict=True)
    yaml_sha = hashlib.sha256(raw).hexdigest()
    image_sha = hashlib.sha256(image.read_bytes()).hexdigest()
    if path == Path(legacy_path).expanduser().resolve():
        if yaml_sha != legacy_yaml_sha or image_sha != legacy_image_sha:
            raise ValueError('Legacy map differs from the confirmed shared map')
    return path, yaml_sha, image_sha
