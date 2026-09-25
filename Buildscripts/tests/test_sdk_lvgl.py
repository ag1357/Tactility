"""Regression coverage for packaging the LVGL ABI used by the firmware build.

Run from the repository root: python -m unittest discover -s Buildscripts/tests
"""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("sdk_release", "Buildscripts/release-sdk-esp32.py")
release = importlib.util.module_from_spec(spec)
spec.loader.exec_module(release)


class LvglSdkTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "managed-lvgl"
        (self.source / "src").mkdir(parents=True)
        (self.source / "lvgl.h").write_text('#include "src/widget.h"\n')
        (self.source / "lv_version.h").write_text('#define LVGL_VERSION_MINOR 3\n')
        (self.source / "src/widget.h").write_text('/* managed 9.3 API */\n')
        (self.source / "Kconfig").write_text(
            'config LV_DRAW_BUF_ALIGN\n int\nconfig LV_USE_MATRIX\n bool\n')
        self.build = self.root / "build"
        (self.build / "config").mkdir(parents=True)
        self.archive = self.build / "liblvgl.a"
        self.archive.write_bytes(b"built-from-managed-9.3")
        (self.build / "project_description.json").write_text(json.dumps({
            "build_component_info": {"lvgl__lvgl": {
                "dir": str(self.source), "file": str(self.archive)}}}))
        (self.build / "config/sdkconfig.h").write_text(
            '#define CONFIG_LV_CONF_SKIP 1\n#define CONFIG_LV_DRAW_BUF_ALIGN 64\n'
            '#define CONFIG_UNRELATED_FIRMWARE_SETTING 123\n')
        self.sdk = self.root / "TactilitySDK"

    def test_uses_compiled_source_and_removes_stale_headers(self):
        stale = self.sdk / "Libraries/lvgl/include/src/newer_only.h"
        stale.parent.mkdir(parents=True)
        stale.write_text("must not survive")
        release.add_lvgl(self.sdk, self.build)
        include = self.sdk / "Libraries/lvgl/include"
        self.assertFalse(stale.exists())
        self.assertIn("MINOR 3", (include / "lv_version.h").read_text())
        self.assertEqual((include / "src/widget.h").read_bytes(),
                         (self.source / "src/widget.h").read_bytes())
        self.assertEqual((self.sdk / "Libraries/lvgl/binary/liblvgl.a").read_bytes(),
                         self.archive.read_bytes())

    @unittest.skipUnless(shutil.which("cc"), "C preprocessor required")
    def test_config_overrides_app_enabled_and_disabled_switches(self):
        release.add_lvgl(self.sdk, self.build)
        include = self.sdk / "Libraries/lvgl/include"
        (include / "esp_attr.h").write_text("")
        program = '''
        #define CONFIG_LV_DRAW_BUF_ALIGN 4
        #define CONFIG_LV_USE_MATRIX 1
        #define CONFIG_UNRELATED_APP_SETTING 19
        #include "tactility_lvgl_sdkconfig.h"
        #if CONFIG_LV_DRAW_BUF_ALIGN != 64
        #error wrong firmware alignment
        #endif
        #ifdef CONFIG_LV_USE_MATRIX
        #error disabled firmware feature leaked from app
        #endif
        #if CONFIG_UNRELATED_APP_SETTING != 19
        #error app configuration was changed
        #endif
        #ifdef CONFIG_UNRELATED_FIRMWARE_SETTING
        #error unrelated firmware setting was copied
        #endif
        '''
        subprocess.run(["cc", "-Werror", "-E", "-x", "c", "-I", str(include), "-"],
                       input=program, text=True, capture_output=True, check=True)

    def test_missing_archive_is_rejected(self):
        self.archive.unlink()
        with self.assertRaises(RuntimeError):
            release.add_lvgl(self.sdk, self.build)

    def test_custom_configuration_is_not_silently_replaced(self):
        (self.build / "config/sdkconfig.h").write_text('#define CONFIG_LV_COLOR_DEPTH 16\n')
        with self.assertRaises(RuntimeError):
            release.add_lvgl(self.sdk, self.build)
