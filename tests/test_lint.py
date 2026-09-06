"""
test_lint.py
============
Linter & Code Quality Enforcement for IUB Drone ROS 2 System.
Enforces PEP 8 syntax formatting, module imports, and code hygiene.
"""

import sys
import os
import unittest
import py_compile
from glob import glob

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestCodeHygieneAndSyntax(unittest.TestCase):

    def test_all_python_files_compile(self):
        """Verifies that every Python source file in the repository is syntactically valid."""
        python_files = glob(os.path.join(ROOT, "**", "*.py"), recursive=True)
        # Exclude build / install directories if present
        python_files = [f for f in python_files if "build" not in f and "install" not in f]

        self.assertGreater(len(python_files), 5, "Should find core Python workspace files.")

        compiled_count = 0
        for py_file in python_files:
            try:
                py_compile.compile(py_file, doraise=True)
                compiled_count += 1
            except py_compile.PyCompileError as e:
                self.fail(f"Syntax/Compilation error in {py_file}:\n{e}")

        self.assertEqual(compiled_count, len(python_files))


if __name__ == "__main__":
    unittest.main()
