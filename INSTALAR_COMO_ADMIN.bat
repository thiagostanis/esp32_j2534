@echo off
setlocal enabledelayedexpansion
title ESP32-J2534 V5 — Build e Instalacao

echo.
echo ============================================================
echo  ESP32-J2534 V5  —  Build automatico da DLL + Instalacao
echo ============================================================
echo.

:: ── Verifica se esta rodando como Administrador ──────────────────
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERRO] Execute este script como Administrador!
    echo        Clique com o botao direito → "Executar como administrador"
    pause
    exit /b 1
)

:: ── Define pasta de destino ───────────────────────────────────────
set INSTALL_DIR=C:\esp32_j2534
set BIN_DIR=%INSTALL_DIR%\bin
set DLL_NAME=esp32_j2534.dll

:: ── Cria estrutura de pastas ──────────────────────────────────────
echo [1/5] Criando pastas em %INSTALL_DIR% ...
if not exist "%INSTALL_DIR%"     mkdir "%INSTALL_DIR%"
if not exist "%BIN_DIR%"         mkdir "%BIN_DIR%"
if not exist "%INSTALL_DIR%\dll" mkdir "%INSTALL_DIR%\dll"

:: ── Copia fontes para a pasta de instalacao ───────────────────────
echo [2/5] Copiando arquivos fonte ...
set SCRIPT_DIR=%~dp0
xcopy /E /I /Y "%SCRIPT_DIR%dll"      "%INSTALL_DIR%\dll\"      >nul
xcopy /E /I /Y "%SCRIPT_DIR%firmware" "%INSTALL_DIR%\firmware\" >nul 2>nul
copy /Y "%SCRIPT_DIR%protocol.h"      "%INSTALL_DIR%\"          >nul
copy /Y "%SCRIPT_DIR%register_j2534.reg" "%INSTALL_DIR%\"       >nul

:: ── Copia config.ini se nao existir (preserva config do usuario) ──
if not exist "%INSTALL_DIR%\config.ini" (
    copy /Y "%SCRIPT_DIR%config.ini" "%INSTALL_DIR%\" >nul
    echo        config.ini copiado. Edite COM_PORT conforme sua porta.
) else (
    echo        config.ini ja existe — preservado sem sobrescrever.
)

:: ── Procura o cmake ───────────────────────────────────────────────
echo [3/5] Procurando CMake e Visual Studio ...
set CMAKE_EXE=
for %%p in (
    "%ProgramFiles%\CMake\bin\cmake.exe"
    "%ProgramFiles(x86)%\CMake\bin\cmake.exe"
    "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    "%ProgramFiles%\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    "%ProgramFiles%\Microsoft Visual Studio\2022\Enterprise\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    "%ProgramFiles(x86)%\Microsoft Visual Studio\2019\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
) do (
    if exist %%p set CMAKE_EXE=%%p
)

:: Tenta cmake no PATH
if "%CMAKE_EXE%"=="" (
    cmake --version >nul 2>&1
    if !errorLevel! equ 0 set CMAKE_EXE=cmake
)

if "%CMAKE_EXE%"=="" (
    echo.
    echo [ERRO] CMake nao encontrado!
    echo        Instale o Visual Studio 2022 com suporte a C++ e CMake, ou
    echo        baixe o CMake em https://cmake.org/download/
    pause
    exit /b 1
)
echo        CMake encontrado: %CMAKE_EXE%

:: ── Detecta Visual Studio (procura vswhere) ───────────────────────
set VSWHERE="%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
set VS_GENERATOR=
if exist %VSWHERE% (
    for /f "usebackq tokens=*" %%i in (`%VSWHERE% -latest -property installationVersion`) do set VS_VER=%%i
    for /f "usebackq tokens=1 delims=." %%i in (`echo !VS_VER!`) do set VS_MAJOR=%%i
    if "!VS_MAJOR!"=="17" set VS_GENERATOR=Visual Studio 17 2022
    if "!VS_MAJOR!"=="16" set VS_GENERATOR=Visual Studio 16 2019
    if "!VS_MAJOR!"=="15" set VS_GENERATOR=Visual Studio 15 2017
)
if "%VS_GENERATOR%"=="" set VS_GENERATOR=Visual Studio 17 2022

echo        Generator: %VS_GENERATOR%

:: ── Build da DLL ─────────────────────────────────────────────────
echo [4/5] Compilando DLL (Win32 / Release) ...
set BUILD_DIR=%INSTALL_DIR%\dll\build_release

if exist "%BUILD_DIR%" rmdir /S /Q "%BUILD_DIR%"
mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"

"%CMAKE_EXE%" "%INSTALL_DIR%\dll" -G "%VS_GENERATOR%" -A Win32 >"%INSTALL_DIR%\build_log.txt" 2>&1
if %errorLevel% neq 0 (
    echo.
    echo [ERRO] CMake configure falhou! Veja: %INSTALL_DIR%\build_log.txt
    pause
    exit /b 1
)

"%CMAKE_EXE%" --build . --config Release >>"%INSTALL_DIR%\build_log.txt" 2>&1
if %errorLevel% neq 0 (
    echo.
    echo [ERRO] Compilacao falhou! Veja: %INSTALL_DIR%\build_log.txt
    pause
    exit /b 1
)

:: ── Copia DLL para bin\ ────────────────────────────────────────────
if exist "%BUILD_DIR%\Release\%DLL_NAME%" (
    copy /Y "%BUILD_DIR%\Release\%DLL_NAME%" "%BIN_DIR%\" >nul
    echo        DLL copiada para %BIN_DIR%\%DLL_NAME%
) else (
    echo [ERRO] DLL nao gerada. Veja %INSTALL_DIR%\build_log.txt
    pause
    exit /b 1
)

:: ── Registro no Windows ───────────────────────────────────────────
echo [5/5] Registrando DLL no Windows Registry ...
reg import "%INSTALL_DIR%\register_j2534.reg" >nul 2>&1
if %errorLevel% neq 0 (
    echo [AVISO] Falha ao importar .reg — tente manualmente:
    echo         reg import "%INSTALL_DIR%\register_j2534.reg"
) else (
    echo        Registry atualizado com sucesso.
)

:: ── Verifica config.ini ───────────────────────────────────────────
echo.
echo ============================================================
echo  INSTALACAO CONCLUIDA!
echo ============================================================
echo.
echo  DLL instalada em: %BIN_DIR%\%DLL_NAME%
echo  Log de build:     %INSTALL_DIR%\build_log.txt
echo.
echo  PROXIMOS PASSOS:
echo  1. Conecte o ESP32 via USB
echo  2. Edite %INSTALL_DIR%\config.ini
echo     Troque COM_PORT=COM5 pela porta real do seu ESP32
echo     (verifique no Gerenciador de Dispositivos)
echo  3. Grave o firmware: %INSTALL_DIR%\firmware\ESP32_J2534_V5.ino
echo  4. Reinicie o Dianalyzer
echo  5. Hardware Options → ESP32 J2534 Bridge → 500 kbps
echo.
echo  Pressione qualquer tecla para abrir o config.ini no Bloco de Notas...
pause >nul
notepad "%INSTALL_DIR%\config.ini"
