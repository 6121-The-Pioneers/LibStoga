@echo off
REM generate_docs.bat - Generate LibStoga Doxygen documentation

echo Generating LibStoga Documentation...

REM Check if Doxygen is installed
where doxygen >nul 2>nul
if %errorlevel% neq 0 (
    echo Doxygen not found. Please install Doxygen first:
    echo    Windows (Chocolatey): choco install doxygen.install
    echo    Windows (Scoop): scoop install doxygen
    echo    Ubuntu/Debian: sudo apt-get install doxygen
    echo    macOS: brew install doxygen
    echo    Download: https://www.doxygen.nl/download.html
    echo.
    echo Press any key to exit...
    pause >nul
    exit /b 1
)

REM Create docs directory if it doesn't exist
if not exist docs mkdir docs

REM Run Doxygen
echo Running Doxygen...
doxygen Doxyfile

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