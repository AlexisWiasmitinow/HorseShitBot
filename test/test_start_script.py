"""Regression tests for side-effect-free start.sh argument validation."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
START_SCRIPT = REPO_ROOT / "scripts" / "start.sh"
BUILDSTART_SCRIPT = REPO_ROOT / "scripts" / "buildstart.sh"
ROBOT_LAUNCH = (
    REPO_ROOT / "src" / "horseshitbot" / "launch" / "robot_launch.py"
)


class StartScriptArgumentTest(unittest.TestCase):
    def run_script(self, *args, script=START_SCRIPT, env=None):
        return subprocess.run(
            ["/bin/bash", str(script), *args],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
            env=env,
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

    def test_buildstart_rejects_unknown_argument_before_build(self):
        result = self.run_script(
            "--definitely-unknown",
            script=BUILDSTART_SCRIPT,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("unknown argument: --definitely-unknown", result.stderr)
        self.assertNotIn("Building workspace", result.stdout + result.stderr)

    def test_startup_scripts_fail_safely_when_stack_is_running(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            fake_pgrep = Path(temp_dir) / "pgrep"
            fake_pgrep.write_text("#!/bin/sh\necho 4242\n")
            fake_pgrep.chmod(0o755)
            env = os.environ.copy()
            env["PATH"] = f"{temp_dir}:{env['PATH']}"

            for script in (START_SCRIPT, BUILDSTART_SCRIPT):
                with self.subTest(script=script.name):
                    result = self.run_script(script=script, env=env)
                    self.assertEqual(result.returncode, 3)
                    self.assertIn("already running", result.stderr)
                    self.assertIn("Stop the existing stack safely", result.stderr)
                    self.assertNotIn("Killing", result.stdout + result.stderr)
                    self.assertNotIn(
                        "Building workspace",
                        result.stdout + result.stderr,
                    )

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
