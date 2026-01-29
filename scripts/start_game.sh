source venv/bin/activate

# Check Python version
current_ver=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
required_ver="3.11"

if [ "$current_ver" != "$required_ver" ]; then
    echo "❌ ERROR: Python version mismatch."
    echo "   Current: $current_ver"
    echo "   Required: $required_ver"
    echo "   This project strictly requires Python 3.11 for Raspberry Pi 5 compatibility."
    echo "   Please create a new venv with Python 3.11 and try again."
    exit 1
fi

python3 main.py