"""Regression tests for side-effect-free start.sh argument validation."""

import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
START_SCRIPT = REPO_ROOT / "scripts" / "start.sh"
ROBOT_LAUNCH = (
    REPO_ROOT / "src" / "horseshitbot" / "launch" / "robot_launch.py"
)


class StartScriptArgumentTest(unittest.TestCase):
    def run_script(self, *args):
        return subprocess.run(
            ["/bin/bash", str(START_SCRIPT), *args],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )

    def test_unknown_argument_fails_before_launch(self):
        result = self.run_script("--definitely-unknown")
        self.assertEqual(result.returncode, 2)
        self.assertIn("unknown argument: --definitely-unknown", result.stderr)
        self.assertNotIn("HorseShitBot —", result.stdout + result.stderr)

    def test_dry_run_is_explicitly_rejected(self):
        result = self.run_script("--dry-run")
        self.assertEqual(result.returncode, 2)
        self.assertIn("--dry-run is not implemented", result.stderr)
        self.assertIn("no processes were started", result.stderr)
        self.assertNotIn("HorseShitBot —", result.stdout + result.stderr)

    def test_help_is_side_effect_free(self):
        result = self.run_script("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("Usage:", result.stdout)
        self.assertIn("--dry-run", result.stdout)
        self.assertIn("--no-battery", result.stdout)
        self.assertNotIn("HorseShitBot —", result.stdout + result.stderr)

    def test_normal_start_enables_shared_bus_battery_monitor(self):
        source = START_SCRIPT.read_text()

        self.assertIn("ENABLE_BATTERY=true", source)
        self.assertIn("--no-battery)  ENABLE_BATTERY=false", source)
        self.assertIn(
            '[ "$ENABLE_MKS" = false ] && ENABLE_BATTERY=false',
            source,
        )
        self.assertIn('enable_battery:="$ENABLE_BATTERY"', source)

    def test_robot_launch_rejects_battery_without_shared_mks_bus(self):
        source = ROBOT_LAUNCH.read_text()

        self.assertIn("if enable_battery and not enable_mks:", source)
        self.assertIn(
            "battery monitor uses mks_bus_node's shared Modbus service",
            source,
        )
        self.assertIn('executable="battery_modbus_node"', source)
        self.assertIn('parameters=[battery_params]', source)


if __name__ == "__main__":
    unittest.main()
