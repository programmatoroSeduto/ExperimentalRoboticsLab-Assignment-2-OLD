#!/bin/bash

sudo apt-get install -y doxygen
sudo apt-get install -y doxygen-gui

pip3 install sphinx==4.5.0
pip3 install breathe
pip3 install sphinx-rtd-theme
pip3 install myst-parser
pip3 install sphinxcontrib-needs
pip3 install sphinxcontrib-plantuml
