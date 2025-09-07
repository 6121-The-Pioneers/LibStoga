@echo off
REM generate_docs.bat - Generate LibStoga Doxygen documentation

set "DOXYGEN_PATH=C:\Program Files\doxygen\bin\doxygen.exe"

echo Generating LibStoga Documentation...

REM Check if Doxygen is installed
if not exist "%DOXYGEN_PATH%" (
    echo Doxygen not found at %DOXYGEN_PATH%.
    echo Please install Doxygen or update the path in this script.
    echo Download: https://www.doxygen.nl/download.html
    echo.
    echo Press any key to exit...
    pause >nul
    exit /b 1
)

REM Create docs directory if it doesn't exist
if not exist docs mkdir docs

REM Run Doxygen
echo Running Doxygen...
"%DOXYGEN_PATH%" Doxyfile

if %errorlevel% equ 0 (
    echo Documentation generated successfully!
    echo Open docs\html\index.html in your browser to view the documentation
    echo.
    echo For online deployment:
    echo - Push to main branch to trigger GitHub Pages deployment
    echo - Documentation will be available at: https://6121-the-pioneers.github.io/LibStoga/
    echo.
    echo Press any key to exit...
    pause >nul
) else (
    echo Documentation generation failed!
    echo.
    echo Press any key to exit...
    pause >nul
    exit /b 1
)