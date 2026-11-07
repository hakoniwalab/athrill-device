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
        self.assertEqual(pixhawk6c["fixture"], "fresh-1-gib-fat32-px4sd")
        self.assertEqual(pixhawk6c["sd_size"], 1_073_741_824)
        self.assertEqual(pixhawk6c["instructions"], 117_119_458)
        self.assertRegex(pixhawk6c["firmware_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk6c["console_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk6c["sd_initial_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(pixhawk6c["sd_sha256"], r"^[0-9a-f]{64}$")

        manifest = RECIPE.RECIPE_FILE.read_text(encoding="utf-8")
        self.assertIn("examples/px4-sih/make-sd-image.sh", manifest)

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
