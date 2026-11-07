import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "recipe.py"
SPEC = importlib.util.spec_from_file_location("athrill_stm32_recipe", SCRIPT)
assert SPEC and SPEC.loader
RECIPE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RECIPE)


class RecipeContractTest(unittest.TestCase):
    def test_source_manifest_has_every_run_path_repository(self) -> None:
        sources = RECIPE.source_pins()
        self.assertEqual(
            set(sources),
            {
                "athrill-device",
                "athrill",
                "athrill-arm-common",
                "athrill-target-stm32",
                "hakoniwa-pdu-registry",
            },
        )
        self.assertEqual(sources["athrill-device"]["revision"], "current-checkout")
        for source_id, source in sources.items():
            if source_id == "athrill-device":
                continue
            self.assertRegex(source["revision"], r"^[0-9a-f]{40}$")
            self.assertTrue(source["repository"].startswith("https://github.com/"))

        manifest = RECIPE.RECIPE_FILE.read_text(encoding="utf-8")
        for source_id, source in sources.items():
            if source_id != "athrill-device":
                self.assertIn(f"revision: {source['revision']}", manifest)

    def test_smoke_oracles_are_measured_values(self) -> None:
        pixhawk4 = RECIPE.smoke_pin("pixhawk4")
        pixhawk6c = RECIPE.smoke_pin("pixhawk6c")
        self.assertEqual(pixhawk4["instructions"], 2_131_907_110)
        self.assertEqual(pixhawk4["success"], "ALL PASS")
        self.assertRegex(pixhawk4["firmware_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk4["console_sha256"], r"^[0-9a-f]{64}$")
        self.assertEqual(
            pixhawk4["required_console_lines"],
            [
                "icm20689 #0 on SPI bus 1 rotation 2",
                "ms5611 #0 on SPI bus 4",
                "ist8310 #0 on I2C bus 3 (external) address 0xE rotation 10",
            ],
        )
        self.assertEqual(pixhawk6c["fixture"], "fresh-1-gib-fat32-px4sd")
        self.assertEqual(pixhawk6c["sd_size"], 1_073_741_824)
        self.assertEqual(pixhawk6c["clock_limit"], 20_000_000_000)
        self.assertEqual(pixhawk6c["instructions"], 872_046_340)
        self.assertRegex(pixhawk6c["firmware_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk6c["console_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk6c["sd_initial_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk6c["sd_sha256"], r"^[0-9a-f]{64}$")
        self.assertEqual(
            pixhawk6c["required_console_lines"],
            [
                "bmi055_accel #0 on SPI bus 1 rotation 4",
                "bmi055_gyro #0 on SPI bus 1 rotation 4",
                "icm42688p #0 on SPI bus 1 rotation 6",
                "ms5611 #0 on I2C bus 4 (external) address 0x77",
                "ist8310 #0 on I2C bus 4 (external) address 0xC",
            ],
        )

        manifest = RECIPE.RECIPE_FILE.read_text(encoding="utf-8")
        self.assertIn("examples/px4-sih/make-sd-image.sh", manifest)

    def test_console_content_check_requires_exact_lines(self) -> None:
        required = ["driver one on SPI1", "driver two on I2C4"]
        with tempfile.TemporaryDirectory(prefix="rc-recipe-console-test.") as temporary:
            console = Path(temporary) / "console.raw"
            console.write_text("driver one on SPI1\r\nnoise\r\ndriver two on I2C4\r\n")
            self.assertEqual(
                RECIPE.verify_console_lines("test board", console, required), required
            )
            console.write_text("driver one on SPI1\n")
            with self.assertRaisesRegex(
                RECIPE.RecipeError, "missing required sensor driver start lines"
            ):
                RECIPE.verify_console_lines("test board", console, required)

    def test_plan_does_not_create_work_directory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="rc-recipe-test.") as temporary:
            work = Path(temporary) / "work"
            result = RECIPE.main([
                "plan", "--board", "pixhawk4", "--work-dir", str(work)
            ])
            self.assertEqual(result, 0)
            self.assertFalse(work.exists())


if __name__ == "__main__":
    unittest.main()
