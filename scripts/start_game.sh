source venv/bin/activate

# Check Python version
current_ver=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
required_ver="3.11"

if [ "$current_ver" != "$required_ver" ]; then
    echo "⚠️  WARNING: You are running Python $current_ver, but this project targets Python $required_ver."
    echo "   Unexpected behavior may occur with PySide6 or depthai on Raspberry Pi."
    echo "   Press ENTER to continue anyway, or Ctrl+C to abort."
    read
fi

python3 main.py