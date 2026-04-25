@echo off

rem  Vivado (TM)
rem  runme.bat: a Vivado-generated Script
rem  Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
rem  Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.


set HD_SDIR=%~dp0
cd /d "%HD_SDIR%"
cscript /nologo /E:JScript "%HD_SDIR%\rundef.js" %*
