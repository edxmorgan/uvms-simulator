"""Sphinx configuration for the UVMS project documentation."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import sys

DOC_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = DOC_DIR.parent
SIMLAB_ROOT = PROJECT_ROOT.parent / "uvms-simlab"

sys.path.insert(0, str(PROJECT_ROOT))
if SIMLAB_ROOT.exists():
    sys.path.insert(0, str(SIMLAB_ROOT))

project = "UVMS Simulator and SimLab"
author = "Edward Morgan"
copyright = f"{datetime.now().year}, {author}"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.napoleon",
]

autosectionlabel_prefix_document = True
templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_title = "UVMS Project Documentation"
html_logo = None
html_theme_options = {
    "collapse_navigation": False,
    "navigation_depth": 3,
    "style_external_links": True,
}

latex_documents = [
    (
        "index",
        "uvms_project_documentation.tex",
        "UVMS Simulator and SimLab Documentation",
        author,
        "manual",
    )
]
latex_elements = {
    "papersize": "letterpaper",
    "pointsize": "10pt",
}
