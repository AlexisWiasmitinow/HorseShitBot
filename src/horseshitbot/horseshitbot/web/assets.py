"""Content-hash versions for the dashboard's code assets.

index.html is served with `/static/app.js?v=<hash>` and
`/static/style.css?v=<hash>`, so a browser that cached an older file (for
example one served before Cache-Control headers existed) gets a new cache key
whenever the content changes and cannot silently keep running old code.
"""

from __future__ import annotations

import hashlib
import threading
from pathlib import Path

VERSIONED_ASSETS = ("app.js", "style.css")
INDEX = "index.html"
VERSION_META = "hsb-frontend-version"


def versioned_index(html: str, asset_versions: dict, frontend_version: str) -> str:
    """Add ?v= to the versioned asset URLs and a version <meta> to the head."""
    for name, version in asset_versions.items():
        html = html.replace(f'"/static/{name}"', f'"/static/{name}?v={version}"')
    meta = f'<meta name="{VERSION_META}" content="{frontend_version}"/>'
    return html.replace("</head>", f"{meta}\n</head>", 1)


class AssetVersions:
    """Short sha256 digests of the static files, re-hashed only when they change."""

    def __init__(self, static_dir: Path):
        self._dir = Path(static_dir)
        self._lock = threading.Lock()
        self._cache: dict[str, tuple[tuple[int, int], str]] = {}

    def _digest(self, name: str) -> str:
        path = self._dir / name
        stat = path.stat()
        key = (stat.st_mtime_ns, stat.st_size)
        with self._lock:
            cached = self._cache.get(name)
            if cached and cached[0] == key:
                return cached[1]
        digest = hashlib.sha256(path.read_bytes()).hexdigest()[:12]
        with self._lock:
            self._cache[name] = (key, digest)
        return digest

    def asset_versions(self) -> dict:
        return {name: self._digest(name) for name in VERSIONED_ASSETS}

    def frontend_version(self) -> str:
        parts = [self._digest(INDEX)] + [self._digest(n) for n in VERSIONED_ASSETS]
        return hashlib.sha256("".join(parts).encode()).hexdigest()[:12]

    def render_index(self) -> str:
        html = (self._dir / INDEX).read_text(encoding="utf-8")
        return versioned_index(html, self.asset_versions(), self.frontend_version())
