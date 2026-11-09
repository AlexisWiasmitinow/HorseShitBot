"""Cache-busted dashboard asset URLs."""

import os
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "horseshitbot"))

from horseshitbot.web.assets import AssetVersions, versioned_index  # noqa: E402

STATIC_DIR = REPO_ROOT / "src" / "horseshitbot" / "horseshitbot" / "web" / "static"

INDEX = """<html><head>
<link href="/static/favicon.png" rel="icon"/>
<link href="/static/style.css" rel="stylesheet"/>
</head><body><script src="/static/app.js"></script></body></html>
"""


class VersionedIndexTest(unittest.TestCase):
    def test_rewrites_code_assets_and_adds_meta(self):
        html = versioned_index(INDEX, {"app.js": "aaa", "style.css": "bbb"}, "fff")
        self.assertIn('src="/static/app.js?v=aaa"', html)
        self.assertIn('href="/static/style.css?v=bbb"', html)
        self.assertIn('href="/static/favicon.png"', html)
        self.assertIn('<meta name="hsb-frontend-version" content="fff"/>\n</head>', html)
        self.assertNotIn('"/static/app.js"', html)


class AssetVersionsTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        (self.dir / "index.html").write_text(INDEX)
        (self.dir / "app.js").write_text("console.log(1);\n")
        (self.dir / "style.css").write_text("body{}\n")
        self.assets = AssetVersions(self.dir)

    def tearDown(self):
        self._tmp.cleanup()

    def _change(self, name, text):
        path = self.dir / name
        stat = path.stat()
        path.write_text(text)
        # Force a different mtime even on coarse-timestamp filesystems.
        os.utime(path, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000_000))

    def test_deterministic(self):
        again = AssetVersions(self.dir)
        self.assertEqual(self.assets.asset_versions(), again.asset_versions())
        self.assertEqual(self.assets.frontend_version(), again.frontend_version())

    def test_js_change_changes_its_url_and_frontend_version(self):
        before_assets = self.assets.asset_versions()
        before = self.assets.frontend_version()
        self._change("app.js", "console.log(2);\n")
        after_assets = self.assets.asset_versions()
        self.assertNotEqual(before_assets["app.js"], after_assets["app.js"])
        self.assertEqual(before_assets["style.css"], after_assets["style.css"])
        self.assertNotEqual(before, self.assets.frontend_version())
        self.assertIn(f'/static/app.js?v={after_assets["app.js"]}', self.assets.render_index())

    def test_index_change_changes_frontend_version(self):
        before = self.assets.frontend_version()
        self._change("index.html", INDEX.replace("<body>", "<body><p>x</p>"))
        self.assertNotEqual(before, self.assets.frontend_version())


class RealIndexTest(unittest.TestCase):
    def test_real_index_references_every_versioned_asset(self):
        html = AssetVersions(STATIC_DIR).render_index()
        self.assertRegex(html, r'src="/static/app\.js\?v=[0-9a-f]{12}"')
        self.assertRegex(html, r'href="/static/style\.css\?v=[0-9a-f]{12}"')
        self.assertRegex(html, r'<meta name="hsb-frontend-version" content="[0-9a-f]{12}"/>')


if __name__ == "__main__":
    unittest.main()
