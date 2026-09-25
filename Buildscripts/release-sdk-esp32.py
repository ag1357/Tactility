#!/usr/bin/env python3

import os
import glob
import subprocess
import sys
import importlib.util
import json
import re
import shutil
from pathlib import Path
from textwrap import dedent

_shared_spec = importlib.util.spec_from_file_location("release_sdk_shared", os.path.join("Buildscripts", "release-sdk-shared.py"))
shared = importlib.util.module_from_spec(_shared_spec)
_shared_spec.loader.exec_module(shared)

def get_driver_mappings(driver_name):
    return [
        {'src': f'Drivers/{driver_name}/include/**', 'dst': f'Drivers/{driver_name}/include/'},
        {'src': f'Drivers/{driver_name}/*.md', 'dst': f'Drivers/{driver_name}/'},
        {'src': f'build/esp-idf/{driver_name}/lib{driver_name}.a', 'dst': f'Drivers/{driver_name}/binary/lib{driver_name}.a'},
    ]

def get_module_mappings(module_name):
    return [
        {'src': f'Modules/{module_name}/include/**', 'dst': f'Modules/{module_name}/include/'},
        {'src': f'Modules/{module_name}/*.md', 'dst': f'Modules/{module_name}/'},
        {'src': f'build/esp-idf/{module_name}/lib{module_name}.a', 'dst': f'Modules/{module_name}/binary/lib{module_name}.a'},
    ]

def create_module_cmakelists(module_name):
    return dedent(f'''
    cmake_minimum_required(VERSION 3.20)
    idf_component_register(
        INCLUDE_DIRS "include"
    )
    add_prebuilt_library({module_name} "binary/lib{module_name}.a")
    ''')

def driver_is_available(driver_name):
    """
    Some drivers only build for certain chip targets (e.g. sc2356-module is ESP32-P4 only,
    since it depends on esp_video/esp_cam_sensor/PPA which are themselves chip-restricted).
    Build output presence is the single source of truth for "does this driver support the
    current target" - no separate manifest to keep in sync with the real CMakeLists.txt
    REQUIRES/Kconfig guards.
    """
    binary_pattern = f'build/esp-idf/{driver_name}/lib{driver_name}.a'
    return bool(glob.glob(binary_pattern))

def add_driver(target_path, driver_name):
    mappings = get_driver_mappings(driver_name)
    shared.map_copy(mappings, target_path)
    cmakelists_content = create_module_cmakelists(driver_name)
    shared.write_module_cmakelists(os.path.join(target_path, f"Drivers/{driver_name}/CMakeLists.txt"), cmakelists_content)

def add_module(target_path, module_name):
    mappings = get_module_mappings(module_name)
    shared.map_copy(mappings, target_path)
    cmakelists_content = create_module_cmakelists(module_name)
    shared.write_module_cmakelists(os.path.join(target_path, f"Modules/{module_name}/CMakeLists.txt"), cmakelists_content)

def add_lvgl(target_path, build_path="build"):
    """Use the headers and Kconfig values that produced the packaged archive."""
    build = Path(build_path)
    description = json.loads((build / "project_description.json").read_text())
    component = description["build_component_info"]["lvgl__lvgl"]
    source = Path(component["dir"])
    archive = Path(component["file"])
    config = (build / "config/sdkconfig.h").read_text()
    definitions = re.findall(r"^#define (CONFIG_LV_\w+)([^\n]*)$", config, re.MULTILINE)
    if not definitions or not archive.is_file() or not (source / "lvgl.h").is_file():
        raise RuntimeError("Build LVGL before generating its SDK headers and configuration")
    if not any(name == "CONFIG_LV_CONF_SKIP" for name, _ in definitions):
        raise RuntimeError("SDK generation requires the firmware's LVGL Kconfig configuration")

    destination = Path(target_path) / "Libraries/lvgl"
    include = destination / "include"
    # A repeated release must not retain headers from a formerly packaged LVGL version.
    if include.exists():
        shutil.rmtree(include)
    include.mkdir(parents=True)
    (destination / "binary").mkdir(exist_ok=True)
    shutil.copy2(archive, destination / "binary/liblvgl.a")
    for name in ("lvgl.h", "lv_version.h"):
        shutil.copy2(source / name, include / name)
    for header in (source / "src").rglob("*.h"):
        output = include / header.relative_to(source)
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(header, output)
    for license_file in source.glob("LICENCE*.*"):
        shutil.copy2(license_file, destination / license_file.name)

    # External apps have their own sdkconfig.h. Reset all LVGL Kconfig switches,
    # including disabled ones, then supply only this archive's enabled values.
    symbols = set(re.findall(r"\bCONFIG_LV_\w+", config))
    for kconfig in source.rglob("Kconfig*"):
        symbols.update("CONFIG_" + name for name in re.findall(
            r"^\s*(?:menu)?config\s+(LV_\w+)", kconfig.read_text(), re.MULTILINE))
    lines = ["/* Generated from the SDK firmware build; do not use app LVGL settings. */",
             "#ifndef TACTILITY_LVGL_SDKCONFIG_H", "#define TACTILITY_LVGL_SDKCONFIG_H",
             '#include "esp_attr.h"']
    lines.extend("#undef " + name for name in sorted(symbols))
    lines.extend("#define " + name + value for name, value in definitions)
    lines.extend(["#endif", ""])
    (include / "tactility_lvgl_sdkconfig.h").write_text("\n".join(lines))

def main():
    if len(sys.argv) < 2:
        print("Usage: release-sdk-esp32.py [target_path]")
        print("Example: release-sdk-esp32.py release/TactilitySDK")
        sys.exit(1)

    esp_idf_version = os.environ.get("ESP_IDF_VERSION", "")
    if not esp_idf_version:
        print("Error: ESP_IDF_VERSION environment variable is not set")
        sys.exit(1)

    target_path = os.path.abspath(sys.argv[1])
    os.makedirs(target_path, exist_ok=True)

    # Mapping logic
    mappings = [
        {'src': 'version.txt', 'dst': ''},
        # TactilityFreeRtos
        {'src': 'TactilityFreeRtos/Include/**', 'dst': 'Libraries/TactilityFreeRtos/Include/'},
        {'src': 'TactilityFreeRtos/CMakeLists.txt', 'dst': 'Libraries/TactilityFreeRtos/'},
        {'src': 'TactilityFreeRtos/LICENSE*.*', 'dst': 'Libraries/TactilityFreeRtos/'},
        # TactilityKernel
        {'src': 'build/esp-idf/TactilityKernel/libTactilityKernel.a', 'dst': 'Libraries/TactilityKernel/binary/'},
        {'src': 'TactilityKernel/include/**', 'dst': 'Libraries/TactilityKernel/include/'},
        {'src': 'TactilityKernel/CMakeLists.txt', 'dst': 'Libraries/TactilityKernel/'},
        {'src': 'TactilityKernel/*.md', 'dst': 'Libraries/TactilityKernel/'},
        # elf_loader
        {'src': 'managed_components/espressif__elf_loader/*.cmake', 'dst': 'Libraries/elf_loader/'},
        {'src': 'managed_components/espressif__elf_loader/*.lf', 'dst': 'Libraries/elf_loader/'},
        {'src': 'managed_components/espressif__elf_loader/license.txt', 'dst': 'Libraries/elf_loader/'},
        # minitar
        {'src': 'build/esp-idf/minitar/libminitar.a', 'dst': 'Libraries/minitar/binary/'},
        {'src': 'Libraries/minitar/minitar/minitar.h', 'dst': 'Libraries/minitar/include/'},
        {'src': 'Libraries/minitar/minitar/LICENSE*', 'dst': 'Libraries/minitar/'},
        # minmea
        {'src': 'build/esp-idf/minmea/libminmea.a', 'dst': 'Libraries/minmea/binary/'},
        {'src': 'Libraries/minmea/Include/**', 'dst': 'Libraries/minmea/include/'},
        {'src': 'Libraries/minmea/CMakeLists.txt', 'dst': 'Libraries/minmea/'},
        {'src': 'Libraries/minmea/README.md', 'dst': 'Libraries/minmea/'},
        {'src': 'Libraries/minmea/LICENSE*.*', 'dst': 'Libraries/minmea/'},
        {'src': 'Libraries/minmea/COPYING', 'dst': 'Libraries/minmea/'},
    ]

    shared.map_copy(mappings, target_path)
    add_lvgl(target_path)

    # Modules
    module_names = shared.read_module_list(os.path.join('Buildscripts', 'release-sdk-modules.txt'))
    for module_name in module_names:
        add_module(target_path, module_name)

    # Final scripts - copied verbatim
    shared.generate_tactility_sdk_cmake(target_path, 'esp32')
    shared.generate_tactility_sdk_top_cmakelists(target_path)

    # Output ESP-IDF SDK version to file
    with open(os.path.join(target_path, "idf-version.txt"), "w") as f:
        f.write(esp_idf_version)

if __name__ == "__main__":
    main()
