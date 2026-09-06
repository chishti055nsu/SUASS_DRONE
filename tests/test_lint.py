"""
test_lint.py
============
Compilation & AST Syntax Integrity Checks for IUB Drone Workspace.
Verifies that all workspace Python files are syntactically valid and parseable into AST.
CI linting (flake8 / pep257 / ament_flake8) is enforced in .github/workflows/ci.yml.
"""

import sys
import os
import unittest
import py_compile
import ast
import subprocess
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

    def test_all_python_files_parse_ast(self):
        """Verifies that every Python file can be parsed into an AST without syntax tree corruption."""
        python_files = glob(os.path.join(ROOT, "**", "*.py"), recursive=True)
        python_files = [f for f in python_files if "build" not in f and "install" not in f]

        for py_file in python_files:
            with open(py_file, "r", encoding="utf-8") as f:
                content = f.read()
            try:
                ast.parse(content, filename=py_file)
            except SyntaxError as e:
                self.fail(f"AST Parsing error in {py_file}:\n{e}")


if __name__ == "__main__":
    unittest.main()
