# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

# Add ROS package path to autodoc
# In docs/source/conf.py
sys.path.insert(0, os.path.abspath('../../cogar_ws/src/assignment1/scripts'))
project = 'Documentation'
copyright = '2025, Mazen Atta & Mazen Madbouly'
author = 'Mazen Atta & Mazen Madbouly'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Extensions
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.imgconverter',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme',
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# Napoleon settings
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True

# Theme
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_show_sourcelink = False

html_context = {
    "display_github": True,    # Enables GitHub ribbon/link
    "github_user": "MazenAtta",    # GitHub username (replace with yours)
    "github_repo": "CogAR_assignment1",      # Repository name (replace with yours)
    "github_version": "main",   # Branch name (main/master)
    "conf_py_path": "/docs/source/",  # Documentation path in repo
}