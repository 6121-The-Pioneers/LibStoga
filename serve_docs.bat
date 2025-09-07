@echo off
echo 🚀 Starting LibStoga Documentation Server with Doxygen Awesome Theme...
echo.

cd /d "%~dp0docs\html"

REM Check if Python 3 is available
python --version >nul 2>&1
if %errorlevel% equ 0 (
    echo 📡 Starting Python HTTP server on port 8000...
    echo 🌐 Open: http://localhost:8000
    echo 📖 Your LibStoga docs now feature the beautiful Doxygen Awesome theme!
    echo.
    python -m http.server 8000
    goto :eof
)

REM Check if Python 2 is available
python2 --version >nul 2>&1
if %errorlevel% equ 0 (
    echo 📡 Starting Python HTTP server on port 8000...
    echo 🌐 Open: http://localhost:8000
    echo 📖 Your LibStoga docs now feature the beautiful Doxygen Awesome theme!
    echo.
    python2 -m SimpleHTTPServer 8000
    goto :eof
)

REM Check if Node.js is available
node --version >nul 2>&1
if %errorlevel% equ 0 (
    echo 📡 Starting Node.js HTTP server on port 8080...
    echo 🌐 Open: http://localhost:8080
    echo 📖 Your LibStoga docs now feature the beautiful Doxygen Awesome theme!
    echo.
    npx http-server -p 8080 -o
    goto :eof
)

echo ❌ No suitable server found. Please install:
echo    - Python (https://python.org)
echo    - Node.js (https://nodejs.org)
echo    - Or use VS Code Live Server extension
echo.
echo 📁 You can also open docs\html\index.html directly in your browser
pause