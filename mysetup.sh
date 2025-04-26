pyenv_local=$(pyenv local 2> /dev/null)
pyenv_status=$?
start_pwd=$(pwd)

## Install meson

pyenv local system
# https://mesonbuild.com/Quick-guide.html#installation-using-python
# sudo apt-get install python3 python3-pip python3-setuptools python3-wheel ninja-build
pip3 install --user meson

## Compile and install libcoinhsl.so (renamed to libhsl.so)

cd coinhsl-archive-2023.11.17
meson setup builddir
cd builddir
meson compile
builddir=$(pwd)
mkdir -p ~/lib
cd ~/lib
ln -s "$builddir/libcoinhsl.so" libhsl.so
cd "$start_pwd"

## Set up project environment

pyenv local 3.12.10

if [ ! -d .venv ]; then
    python3 -m venv .venv
fi
source .venv/bin/activate
pip3 install -r requirements.txt

## Restore previous shell state

if [ $pyenv_status -eq 0 ]; then
    pyenv local $pyenv_local  # Set the local pyenv to what it was before
else
    rm .python-version  # Remove local pyenv version file
fi

cd "$start_pwd"
