@echo off
git submodule update --init --recursive
tools\premake5 %* vs2022 --copy-to="D:\SteamLibrary\steamapps\common\s1x_full_game"