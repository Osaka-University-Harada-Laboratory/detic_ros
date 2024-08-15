# In Ubuntu, the command for the base installation of Python 3 is python3.
# Also, pip and venv may need to be installed manually before running this
# script since it may not come with the base installation of Python 3.
# To get pip: sudo apt install python3-pip
# To get venv: sudo apt install python3-venv
# To get doxygen: sudo apt install doxygen

# Get the directory where this script lives.
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"

# Check that the k4a.so and depth_engine.so have been copied into the _libs/ folder.
# Because these dlls may not be located in a standard place, leave it to the developer
# to manually copy them into the _libs/ folder.
if [ ! -f "$DIR/src/k4a/_libs/libk4a.so" ] && [ ! -h "$DIR/src/k4a/_libs/libk4a.so" ]; then
    echo "File not found: $DIR/src/k4a/_libs/libk4a.so"
    echo "Please manually copy the k4a library into that folder."
    return 1
fi

if [ ! -f "$DIR/src/k4a/_libs/libdepthengine"* ] && [ ! -h "$DIR/src/k4a/_libs/libdepthengine"* ]; then
    echo "File not found: $DIR/src/k4a/_libs/libdepthengine*"
    echo "Please manually copy the depth engine library into that folder."
    return 1
fi

# Create a virtual environment and activate it.
if [ -d "build" ]; then
    rm -rf "build"
fi

# Install the package in editable mode so that it installs dependencies. These are needed for sphinx docs.
pip3 install --upgrade pip
pip3 install -e .

# Build the .whl file and place it in a build/ folder.
pip3 install wheel
pip3 wheel . -w build

echo "Done creating k4a package wheel."
