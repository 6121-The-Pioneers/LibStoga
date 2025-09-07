#!/bin/bash
# generate_docs.sh - Cross-platform script to generate LibStoga Doxygen documentation

echo "🤖 Generating LibStoga Documentation..."

# Check if Doxygen is installed
if ! command -v doxygen &> /dev/null; then
    echo "❌ Doxygen not found. Please install Doxygen first:"
    echo "   Windows (Chocolatey): choco install doxygen.install"
    echo "   Windows (Scoop): scoop install doxygen"
    echo "   Ubuntu/Debian: sudo apt-get install doxygen"
    echo "   macOS: brew install doxygen"
    echo "   Or download from: https://www.doxygen.nl/download.html"
    exit 1
fi

# Create docs directory if it doesn't exist
mkdir -p docs

# Run Doxygen
echo "📚 Running Doxygen..."
doxygen Doxyfile

if [ $? -eq 0 ]; then
    echo "✅ Documentation generated successfully!"
    echo "📁 Open docs/html/index.html in your browser to view the documentation"
    echo ""
    echo "🌐 For online deployment:"
    echo "   - Push to main branch to trigger GitHub Pages deployment"
    echo "   - Documentation will be available at: https://6121-the-pioneers.github.io/LibStoga/"
else
    echo "❌ Documentation generation failed!"
    exit 1
fi