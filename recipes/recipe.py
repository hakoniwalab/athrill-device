#!/usr/bin/env python3
"""Component-owned runner for the Athrill STM32 board build Recipe."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


REPOSITORY = Path(__file__).resolve().parents[1]
RECIPE_ID = "athrill-stm32-board-build"
RECIPE_FILE = REPOSITORY / "recipes" / f"{RECIPE_ID}.yaml"
SOURCE_PINS = REPOSITORY / "recipes" / "pins" / "sources.json"
SMOKE_PINS = REPOSITORY / "recipes" / "pins" / "smoke.json"
SOURCE_ENV = {
    "athrill": "ATHRILL_ROOT",
    "athrill-arm-common": "ATHRILL_ARM_COMMON_ROOT",
    "athrill-target-stm32": "ATHRILL_TARGET_STM32_ROOT",
    "hakoniwa-pdu-registry": "HAKONIWA_PDU_REGISTRY_ROOT",
}


class RecipeError(RuntimeError):
    pass


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RecipeError(f"expected a JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_text(command: Sequence[object]) -> str:
    return " ".join(str(part) for part in command)


def run_command(
    command: Sequence[object],
    *,
    cwd: Path,
    env: Mapping[str, str] | None = None,
    check: bool = True,
    capture: bool = False,
) -> subprocess.CompletedProcess[str]:
    rendered = command_text(command)
    print(f"> {rendered}", flush=True)
    result = subprocess.run(
        [str(part) for part in command],
        cwd=cwd,
        env=None if env is None else dict(env),
        text=True,
        capture_output=capture,
        check=False,
    )
    if capture:
        if result.stdout:
            print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
        if result.stderr:
            print(result.stderr, end="" if result.stderr.endswith("\n") else "\n", file=sys.stderr)
    if check and result.returncode:
        raise RecipeError(f"command failed with exit={result.returncode}: {rendered}")
    return result


def git_revision(path: Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=path, text=True,
        capture_output=True, check=False,
    )
    if result.returncode:
        raise RecipeError(f"not a Git checkout: {path}")
    return result.stdout.strip()


def git_dirty(path: Path) -> bool:
    result = subprocess.run(
        ["git", "status", "--porcelain"], cwd=path, text=True,
        capture_output=True, check=False,
    )
    return bool(result.stdout.strip())


def resolve_work_dir(value: str | None) -> Path:
    if value:
        return Path(value).expanduser().resolve()
    from_env = os.environ.get("HAKONIWA_RECIPE_WORK_DIR", "").strip()
    if from_env:
        return Path(from_env).expanduser().resolve()
    return (REPOSITORY / "work" / "recipes" / RECIPE_ID).resolve()


def resolve_business_pack(value: str | None) -> Path | None:
    selected = value or os.environ.get("HAKONIWA_BUSINESS_PACK_ROOT", "")
    return Path(selected).expanduser().resolve() if selected.strip() else None


def resolve_foundation(value: str | None, business_pack: Path | None) -> Path | None:
    selected = value or os.environ.get("HAKONIWA_FOUNDATION_PREFIX", "")
    if selected.strip():
        return Path(selected).expanduser().resolve()
    if business_pack is not None:
        return (business_pack / "work" / "foundation" / "install").resolve()
    return None


def source_paths(work: Path) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for source_id, env_name in SOURCE_ENV.items():
        override = os.environ.get(env_name, "").strip()
        result[source_id] = (
            Path(override).expanduser().resolve()
            if override else work / "sources" / source_id
        )
    return result


def source_pins() -> dict[str, dict[str, str]]:
    raw = load_json(SOURCE_PINS).get("sources")
    if not isinstance(raw, dict):
        raise RecipeError(f"sources mapping is missing: {SOURCE_PINS}")
    return raw


def smoke_pin(board: str) -> dict[str, Any]:
    boards = load_json(SMOKE_PINS).get("boards")
    if not isinstance(boards, dict) or board not in boards:
        raise RecipeError(f"smoke pin is missing for {board}: {SMOKE_PINS}")
    return boards[board]


def require_foundation(prefix: Path | None) -> list[str]:
    if prefix is None:
        return ["Foundation prefix is not selected (--foundation-prefix or --business-pack-root)"]
    required = (
        prefix / "lib" / "cmake" / "hakoniwa-core" / "hakoniwa-coreConfig.cmake",
        prefix / "lib" / ("libshakoc.dylib" if platform.system() == "Darwin" else "libshakoc.so"),
        prefix / "share" / "hakoniwa" / "receipts" / "hakoniwa-core-pro.yaml",
    )
    return [f"missing Foundation artifact: {path}" for path in required if not path.exists()]


def inspect_sources(work: Path) -> list[str]:
    problems: list[str] = []
    pins = source_pins()
    for source_id, path in source_paths(work).items():
        pin = pins[source_id]
        if not path.is_dir():
            problems.append(f"missing source {source_id}: {path}")
            continue
        required = path / pin["required"]
        if not required.exists():
            problems.append(f"missing source artifact for {source_id}: {required}")
            continue
        try:
            actual = git_revision(path)
        except RecipeError as error:
            problems.append(str(error))
            continue
        if actual != pin["revision"]:
            problems.append(
                f"source revision mismatch for {source_id}: expected {pin['revision']}, got {actual}"
            )
    return problems


def verify_target_ci_pins(target: Path) -> None:
    workflow = (target / ".github" / "workflows" / "ctest.yml").read_text(encoding="utf-8")
    pins = source_pins()
    for source_id in ("athrill", "athrill-arm-common"):
        revision = pins[source_id]["revision"]
        if revision not in workflow:
            raise RecipeError(
                f"{source_id} pin {revision} does not match athrill-target-stm32 CI"
            )


def configure_sources(work: Path) -> None:
    work.joinpath("sources").mkdir(parents=True, exist_ok=True)
    for source_id, destination in source_paths(work).items():
        pin = source_pins()[source_id]
        override = bool(os.environ.get(SOURCE_ENV[source_id], "").strip())
        if destination.exists():
            if not destination.is_dir():
                raise RecipeError(f"source path is not a directory: {destination}")
            actual = git_revision(destination)
            if actual != pin["revision"]:
                label = "override" if override else "existing managed source"
                raise RecipeError(
                    f"{label} has the wrong revision for {source_id}: "
                    f"expected {pin['revision']}, got {actual}; it was not modified"
                )
        else:
            if override:
                raise RecipeError(
                    f"override path does not exist for {source_id}: {destination}"
                )
            destination.parent.mkdir(parents=True, exist_ok=True)
            run_command(["git", "clone", pin["repository"], destination], cwd=work)
            run_command(["git", "checkout", "--detach", pin["revision"]], cwd=destination)
        required = destination / pin["required"]
        if not required.exists():
            raise RecipeError(f"required source artifact is missing: {required}")
    verify_target_ci_pins(source_paths(work)["athrill-target-stm32"])


def configure_foundation(
    business_pack: Path | None, prefix: Path | None, work: Path,
) -> None:
    problems = require_foundation(prefix)
    if not problems:
        return
    if business_pack is None:
        raise RecipeError("\n".join(problems + [
            "select --business-pack-root so configure can install hakoniwa-core-pro",
        ]))
    entrypoint = business_pack / "tools" / "recipe.py"
    if not entrypoint.is_file():
        raise RecipeError(f"Business Pack Recipe entry point is missing: {entrypoint}")
    env = dict(os.environ)
    for source_id, path in source_paths(work).items():
        env[SOURCE_ENV[source_id]] = str(path)
    run_command(
        [sys.executable, entrypoint, "configure", "--recipe", RECIPE_FILE],
        cwd=business_pack, env=env,
    )
    remaining = require_foundation(prefix)
    if remaining:
        raise RecipeError("\n".join(remaining))


def doctor(args: argparse.Namespace) -> int:
    work = resolve_work_dir(args.work_dir)
    business_pack = resolve_business_pack(args.business_pack_root)
    foundation = resolve_foundation(args.foundation_prefix, business_pack)
    problems: list[str] = []
    for tool in ("cmake", "git"):
        if shutil.which(tool) is None:
            problems.append(f"host tool is missing: {tool}")
    problems.extend(require_foundation(foundation))
    problems.extend(inspect_sources(work))
    if args.firmware:
        firmware = Path(args.firmware).expanduser().resolve()
        if not firmware.is_file():
            problems.append(f"firmware ELF is missing: {firmware}")
        else:
            expected = smoke_pin(args.board)["firmware_sha256"]
            actual = sha256(firmware)
            if actual != expected:
                problems.append(
                    f"firmware SHA-256 mismatch for {args.board}: "
                    f"expected {expected}, got {actual}"
                )
    if args.hako_drone_root and not args.hako_drone_source_dir:
        problems.append("--hako-drone-source-dir is required with --hako-drone-root")
    status = "SATISFIED" if not problems else "MISSING"
    print(f"{RECIPE_ID}: {status}")
    print(f"  board: {args.board}")
    print(f"  work: {work}")
    print(f"  foundation: {foundation or 'not selected'}")
    for problem in problems:
        print(f"  - {problem}")
    return 0 if not problems else 1


def plan(args: argparse.Namespace) -> int:
    work = resolve_work_dir(args.work_dir)
    business_pack = resolve_business_pack(args.business_pack_root)
    foundation = resolve_foundation(args.foundation_prefix, business_pack)
    paths = source_paths(work)
    pins = source_pins()
    print(f"Recipe: {RECIPE_ID}")
    print(f"Board: {args.board}")
    print(f"Work directory: {work}")
    print("Sources:")
    for source_id, path in paths.items():
        action = "reuse" if path.exists() else "clone"
        print(f"  {source_id}: {action} {path} @ {pins[source_id]['revision']}")
    print(f"  athrill-device: reuse {REPOSITORY} @ current checkout")
    print("Packages:")
    foundation_state = "reuse" if not require_foundation(foundation) else "configure"
    print(f"  hakoniwa-core-pro: {foundation_state} {foundation or 'through Business Pack'}")
    print("Build options:")
    print("  athrill3: Release, LTO=ON, EXDEV=ON, DEVICE_TREE=ON, ATHRILL_ADDRESS_WIDTH=32")
    print("  hako_sensor_device: Release, ATHRILL_ADDRESS_WIDTH=32")
    if args.hako_drone_root:
        print("  hako_sensor_device_inproc: enabled (macOS Hakoniwa Drone v5 release)")
    else:
        print("  hako_sensor_device_inproc: omitted (no Hakoniwa Drone release root)")
    smoke = smoke_pin(args.board)
    print(f"Smoke: {args.board} DT path with {smoke['dtb']}")
    print(f"  firmware SHA-256: {smoke['firmware_sha256']}")
    print(f"  expected instructions: {smoke['instructions']}")
    print(f"  expected console SHA-256: {smoke['console_sha256']}")
    print("  required console lines:")
    for line in smoke["required_console_lines"]:
        print(f"    - {line}")
    if smoke.get("sd_initial_sha256"):
        print(f"  SD fixture: {smoke['fixture']}")
        print(f"  initial SD SHA-256: {smoke['sd_initial_sha256']}")
        print(f"  expected post-smoke SD SHA-256: {smoke['sd_sha256']}")
    return 0


def configure(args: argparse.Namespace) -> int:
    work = resolve_work_dir(args.work_dir)
    business_pack = resolve_business_pack(args.business_pack_root)
    foundation = resolve_foundation(args.foundation_prefix, business_pack)
    work.mkdir(parents=True, exist_ok=True)
    configure_sources(work)
    configure_foundation(business_pack, foundation, work)
    config = work / "config"
    config.mkdir(parents=True, exist_ok=True)
    inputs = {
        "schema_version": 1,
        "foundation_prefix": str(foundation),
        "hako_drone_root": str(Path(args.hako_drone_root).expanduser().resolve()) if args.hako_drone_root else None,
        "hako_drone_source_dir": str(Path(args.hako_drone_source_dir).expanduser().resolve()) if args.hako_drone_source_dir else None,
    }
    config.joinpath("inputs.json").write_text(
        json.dumps(inputs, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"Configured sources and inputs under {work}")
    return 0


def stored_inputs(work: Path) -> dict[str, Any]:
    path = work / "config" / "inputs.json"
    return load_json(path) if path.is_file() else {}


def build(args: argparse.Namespace) -> int:
    work = resolve_work_dir(args.work_dir)
    problems = inspect_sources(work)
    if problems:
        raise RecipeError("\n".join(problems))
    paths = source_paths(work)
    inputs = stored_inputs(work)
    business_pack = resolve_business_pack(args.business_pack_root)
    foundation = resolve_foundation(args.foundation_prefix, business_pack)
    if foundation is None and inputs.get("foundation_prefix"):
        foundation = Path(inputs["foundation_prefix"])
    foundation_problems = require_foundation(foundation)
    if foundation_problems:
        raise RecipeError("\n".join(foundation_problems))
    jobs = str(args.jobs or max(1, os.cpu_count() or 1))
    build_root = work / "build"
    target_build = build_root / "athrill3"
    plugin_build = build_root / "sensor-device"
    build_root.mkdir(parents=True, exist_ok=True)
    run_command([
        "cmake", "-S", paths["athrill-target-stm32"], "-B", target_build,
        "-DCMAKE_BUILD_TYPE=Release",
        "-DATHRILL_ENABLE_LTO=ON",
        "-DATHRILL_ENABLE_EXDEV=ON",
        "-DATHRILL_ENABLE_DEVICE_TREE=ON",
        f"-DATHRILL_SOURCE_DIR={paths['athrill']}",
        f"-DATHRILL_ARM_COMMON_DIR={paths['athrill-arm-common']}",
        f"-DATHRILL_DEVICE_DIR={REPOSITORY}",
    ], cwd=work)
    run_command(["cmake", "--build", target_build, "--target", "athrill3", "--parallel", jobs], cwd=work)

    drone_root = args.hako_drone_root or inputs.get("hako_drone_root")
    drone_source = args.hako_drone_source_dir or inputs.get("hako_drone_source_dir")
    if bool(drone_root) != bool(drone_source):
        raise RecipeError("Hakoniwa Drone release root and matching source dir must be provided together")
    plugin_command: list[object] = [
        "cmake", "-S", REPOSITORY / "board" / "common", "-B", plugin_build,
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_PREFIX_PATH={foundation}",
        f"-DATHRILL_SOURCE_DIR={paths['athrill']}",
        "-DATHRILL_ADDRESS_WIDTH=32",
        f"-DHAKO_PDU_REGISTRY_DIR={paths['hakoniwa-pdu-registry']}",
        f"-DHAKO_SENSOR_DEVICE_BUILD_INPROC={'ON' if drone_root else 'OFF'}",
    ]
    if drone_root:
        plugin_command.extend([
            f"-DHAKO_DRONE_ROOT={Path(drone_root).resolve()}",
            f"-DHAKO_DRONE_SOURCE_DIR={Path(drone_source).resolve()}",
        ])
    run_command(plugin_command, cwd=work)
    run_command(["cmake", "--build", plugin_build, "--parallel", jobs], cwd=work)

    metadata = {
        "schema_version": 1,
        "build_type": "Release",
        "athrill_enable_lto": True,
        "athrill_enable_exdev": True,
        "athrill_enable_device_tree": True,
        "athrill_address_width": 32,
        "in_process_plugin": bool(drone_root),
        "foundation_prefix": str(foundation),
    }
    work.joinpath("build-metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


def parse_steps(text: str) -> int:
    matches = re.findall(r"steps=(\d+)", text)
    if not matches:
        matches = re.findall(r"\((\d+) instructions\)", text)
    if not matches:
        raise RecipeError("Athrill did not report an instruction count (steps=...)")
    return int(matches[-1])


def verify_console_lines(board: str, console: Path, required: Sequence[str]) -> list[str]:
    if not console.is_file():
        raise RecipeError(f"{board} UART console capture was not produced: {console}")
    actual = set(console.read_text(encoding="utf-8", errors="replace").splitlines())
    missing = [line for line in required if line not in actual]
    if missing:
        rendered = "\n".join(f"  - {line}" for line in missing)
        raise RecipeError(
            f"{board} console is missing required sensor driver start lines:\n{rendered}"
        )
    return list(required)


def smoke_pixhawk4(work: Path, firmware: Path, athrill3: Path) -> dict[str, Any]:
    target = source_paths(work)["athrill-target-stm32"]
    pin = smoke_pin("pixhawk4")
    dtb = REPOSITORY / pin["dtb"]
    env = dict(os.environ)
    env["DEVICE_CONFIG_DEVICE_TREE"] = str(dtb)
    result = run_command(
        [target / "examples" / "px4-bench" / "check.sh", firmware, athrill3],
        cwd=target, env=env, capture=True, check=False,
    )
    output = result.stdout + result.stderr
    instructions = parse_steps(output)
    if result.returncode or pin["success"] not in output:
        raise RecipeError("Pixhawk 4 benchmark did not report ALL PASS")
    if instructions != pin["instructions"]:
        raise RecipeError(
            f"Pixhawk 4 instruction count mismatch: expected {pin['instructions']}, got {instructions}"
        )
    console = target / "examples" / "px4-bench" / "work" / "run1" / "console.txt"
    console_lines = verify_console_lines("Pixhawk 4", console, pin["required_console_lines"])
    console_hash = sha256(console)
    if console_hash != pin["console_sha256"]:
        raise RecipeError(
            f"Pixhawk 4 console hash mismatch: expected {pin['console_sha256']}, got {console_hash}"
        )
    return {
        "status": "verified",
        "board": "pixhawk4",
        "dtb": str(dtb),
        "instructions": instructions,
        "expected_instructions": pin["instructions"],
        "success": pin["success"],
        "console_sha256": console_hash,
        "expected_console_sha256": pin["console_sha256"],
        "required_console_lines": console_lines,
        "firmware_sha256": sha256(firmware),
    }


def smoke_pixhawk6c(work: Path, firmware: Path, athrill3: Path) -> dict[str, Any]:
    target = source_paths(work)["athrill-target-stm32"]
    pin = smoke_pin("pixhawk6c")
    smoke = work / "smoke" / "pixhawk6c"
    if smoke.exists():
        if work not in smoke.parents:
            raise RecipeError(f"refusing to replace smoke directory outside Recipe work: {smoke}")
        shutil.rmtree(smoke)
    smoke.mkdir(parents=True)
    example = target / "examples" / "px4-v6c-boot"
    for name in ("memory.txt", "device_config.txt", "shell.nsh"):
        shutil.copy2(example / name, smoke / name)
    sd_image = smoke / "sd.img"
    sd_generator = target / "examples" / "px4-sih" / "make-sd-image.sh"
    if not sd_generator.is_file():
        raise RecipeError(f"Pixhawk 6C SD image generator is missing: {sd_generator}")
    run_command([sd_generator, sd_image], cwd=smoke)
    if not sd_image.is_file():
        raise RecipeError("Pixhawk 6C SD image generator did not produce sd.img")
    if sd_image.stat().st_size != pin["sd_size"]:
        raise RecipeError(
            f"Pixhawk 6C initial SD size mismatch: "
            f"expected {pin['sd_size']}, got {sd_image.stat().st_size}"
        )
    initial_sd_hash = sha256(sd_image)
    if initial_sd_hash != pin["sd_initial_sha256"]:
        raise RecipeError(
            f"Pixhawk 6C initial SD hash mismatch: "
            f"expected {pin['sd_initial_sha256']}, got {initial_sd_hash}"
        )
    dtb = REPOSITORY / pin["dtb"]
    with smoke.joinpath("device_config.txt").open("a", encoding="utf-8") as stream:
        stream.write(f"DEVICE_CONFIG_DEVICE_TREE {dtb}\n")
    result = run_command([
        athrill3, "-c1", "-m", "memory.txt", "-d", "device_config.txt",
        "-t", str(pin["clock_limit"]), firmware,
    ], cwd=smoke, capture=True, check=False)
    smoke.joinpath("stdout.txt").write_text(result.stdout, encoding="utf-8")
    smoke.joinpath("stderr.txt").write_text(result.stderr, encoding="utf-8")
    if result.returncode != 1 or "EXIT for timeout" not in result.stdout + result.stderr:
        raise RecipeError(
            f"Pixhawk 6C bounded boot expected timeout exit 1, got {result.returncode}"
        )
    instructions = parse_steps(result.stdout + result.stderr)
    console = smoke / "console.raw"
    console_lines = verify_console_lines("Pixhawk 6C", console, pin["required_console_lines"])
    console_hash = sha256(console)
    sd_hash = sha256(sd_image)
    if instructions != pin["instructions"]:
        raise RecipeError(
            f"Pixhawk 6C instruction count mismatch: expected {pin['instructions']}, got {instructions}"
        )
    if console_hash != pin["console_sha256"]:
        raise RecipeError(
            f"Pixhawk 6C console hash mismatch: expected {pin['console_sha256']}, got {console_hash}"
        )
    if sd_hash != pin["sd_sha256"]:
        raise RecipeError(
            f"Pixhawk 6C SD hash mismatch: expected {pin['sd_sha256']}, got {sd_hash}"
        )
    return {
        "status": "verified",
        "board": "pixhawk6c",
        "fixture": pin["fixture"],
        "dtb": str(dtb),
        "clock_limit": pin["clock_limit"],
        "instructions": instructions,
        "expected_instructions": pin["instructions"],
        "console_sha256": console_hash,
        "expected_console_sha256": pin["console_sha256"],
        "required_console_lines": console_lines,
        "sd_initial_sha256": initial_sd_hash,
        "expected_sd_initial_sha256": pin["sd_initial_sha256"],
        "sd_sha256": sd_hash,
        "expected_sd_sha256": pin["sd_sha256"],
        "firmware_sha256": sha256(firmware),
    }


def smoke(args: argparse.Namespace) -> int:
    if not args.firmware:
        raise RecipeError("--firmware is required for smoke")
    firmware = Path(args.firmware).expanduser().resolve()
    if not firmware.is_file():
        raise RecipeError(f"firmware ELF is missing: {firmware}")
    expected_firmware = smoke_pin(args.board)["firmware_sha256"]
    actual_firmware = sha256(firmware)
    if actual_firmware != expected_firmware:
        raise RecipeError(
            f"firmware SHA-256 mismatch for {args.board}: "
            f"expected {expected_firmware}, got {actual_firmware}"
        )
    work = resolve_work_dir(args.work_dir)
    athrill3 = work / "build" / "athrill3" / "athrill3"
    if not athrill3.is_file():
        raise RecipeError(f"athrill3 is not built: {athrill3}")
    result = (
        smoke_pixhawk4(work, firmware, athrill3)
        if args.board == "pixhawk4"
        else smoke_pixhawk6c(work, firmware, athrill3)
    )
    validation = work / "validation"
    validation.mkdir(parents=True, exist_ok=True)
    path = validation / f"{args.board}.json"
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def dylib_name(base: str) -> str:
    if platform.system() == "Darwin":
        return f"lib{base}.dylib"
    if platform.system() == "Windows":
        return f"{base}.dll"
    return f"lib{base}.so"


def yaml_string(value: object) -> str:
    return json.dumps(str(value))


def write_receipt(path: Path, data: dict[str, Any]) -> None:
    lines = [
        "schema_version: 1",
        "recipe:",
        f"  id: {data['recipe']['id']}",
        f"  board: {data['recipe']['board']}",
        "platform:",
        f"  os: {yaml_string(data['platform']['os'])}",
        f"  architecture: {yaml_string(data['platform']['architecture'])}",
        "sources:",
    ]
    for source_id, source in data["sources"].items():
        lines.extend([
            f"  {source_id}:",
            f"    revision: {yaml_string(source['revision'])}",
            f"    dirty: {'true' if source['dirty'] else 'false'}",
        ])
    lines.extend([
        "build_options:",
        "  build_type: Release",
        "  lto: true",
        "  exdev: true",
        "  device_tree: true",
        "  athrill_address_width: 32",
        f"  in_process_plugin: {'true' if data['build_options']['in_process_plugin'] else 'false'}",
        "smoke:",
        f"  status: {data['smoke']['status']}",
        f"  instructions: {data['smoke']['instructions']}",
        f"  console_sha256: {yaml_string(data['smoke']['console_sha256'])}",
        f"  firmware_sha256: {yaml_string(data['smoke']['firmware_sha256'])}",
    ])
    if data["smoke"].get("fixture"):
        lines.append(f"  fixture: {yaml_string(data['smoke']['fixture'])}")
    if data["smoke"].get("sd_initial_sha256"):
        lines.append(
            f"  sd_initial_sha256: {yaml_string(data['smoke']['sd_initial_sha256'])}"
        )
    if data["smoke"].get("sd_sha256"):
        lines.append(f"  sd_sha256: {yaml_string(data['smoke']['sd_sha256'])}")
    if data["smoke"].get("required_console_lines"):
        lines.extend([
            "  console_content:",
            "    status: verified",
            "    required_lines:",
        ])
        for line in data["smoke"]["required_console_lines"]:
            lines.append(f"      - {yaml_string(line)}")
    lines.append("artifacts:")
    for artifact in data["artifacts"]:
        lines.extend([
            f"  - path: {yaml_string(artifact['path'])}",
            f"    sha256: {yaml_string(artifact['sha256'])}",
        ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def install(args: argparse.Namespace) -> int:
    work = resolve_work_dir(args.work_dir)
    build_metadata = load_json(work / "build-metadata.json")
    validation = load_json(work / "validation" / f"{args.board}.json")
    paths = source_paths(work)
    install_root = work / "install"
    bin_dir = install_root / "bin"
    lib_dir = install_root / "lib"
    boards_dir = install_root / "boards"
    receipt_dir = install_root / "share" / "hakoniwa" / "receipts"
    for directory in (bin_dir, lib_dir, boards_dir, receipt_dir):
        directory.mkdir(parents=True, exist_ok=True)
    artifacts: list[Path] = []
    athrill3 = work / "build" / "athrill3" / "athrill3"
    destination = bin_dir / athrill3.name
    shutil.copy2(athrill3, destination)
    artifacts.append(destination)
    plugin_build = work / "build" / "sensor-device"
    for base in ("hako_sensor_device", "hako_sensor_device_inproc"):
        source = plugin_build / dylib_name(base)
        if source.is_file():
            destination = lib_dir / source.name
            shutil.copy2(source, destination)
            artifacts.append(destination)
    for board in ("pixhawk4", "pixhawk6c"):
        destination_board = boards_dir / board
        destination_board.mkdir(parents=True, exist_ok=True)
        for suffix in (".dtb", "-hakoniwa.dtb"):
            source = REPOSITORY / "boards" / board / f"{board}{suffix}"
            destination = destination_board / source.name
            shutil.copy2(source, destination)
            artifacts.append(destination)
    sources: dict[str, dict[str, Any]] = {}
    for source_id, source_path in paths.items():
        sources[source_id] = {
            "revision": git_revision(source_path),
            "dirty": git_dirty(source_path),
        }
    sources["athrill-device"] = {
        "revision": git_revision(REPOSITORY),
        "dirty": git_dirty(REPOSITORY),
    }
    receipt_data = {
        "recipe": {"id": RECIPE_ID, "board": args.board},
        "platform": {"os": platform.system().lower(), "architecture": platform.machine()},
        "sources": sources,
        "build_options": build_metadata,
        "smoke": validation,
        "artifacts": [
            {"path": str(path.relative_to(install_root)), "sha256": sha256(path)}
            for path in artifacts
        ],
    }
    receipt = receipt_dir / f"{RECIPE_ID}-{args.board}.yaml"
    write_receipt(receipt, receipt_data)
    print(f"Installed artifacts under {install_root}")
    print(f"Receipt: {receipt}")
    return 0


def run_all(args: argparse.Namespace) -> int:
    configure(args)
    if doctor(args):
        raise RecipeError("doctor failed after configure")
    build(args)
    smoke(args)
    install(args)
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "operation", choices=("doctor", "plan", "configure", "build", "smoke", "install", "run")
    )
    result.add_argument("--recipe", type=Path, default=RECIPE_FILE)
    result.add_argument("--board", required=True, choices=("pixhawk4", "pixhawk6c"))
    result.add_argument("--firmware", help="PX4 v1.17.0 board ELF path")
    result.add_argument("--work-dir", help="Recipe work directory")
    result.add_argument("--business-pack-root", help="Hakoniwa Business Pack checkout")
    result.add_argument("--foundation-prefix", help="Installed Hakoniwa Foundation prefix")
    result.add_argument("--jobs", type=int, help="Parallel build jobs")
    result.add_argument("--hako-drone-root", help="Optional unpacked Hakoniwa Drone v5 release root")
    result.add_argument("--hako-drone-source-dir", help="Matching optional Hakoniwa Drone v5 source tree")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = parser().parse_args(argv)
    if args.recipe.resolve() != RECIPE_FILE.resolve():
        raise RecipeError(f"unsupported Recipe manifest: {args.recipe}")
    operations = {
        "doctor": doctor,
        "plan": plan,
        "configure": configure,
        "build": build,
        "smoke": smoke,
        "install": install,
        "run": run_all,
    }
    try:
        return operations[args.operation](args)
    except RecipeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
