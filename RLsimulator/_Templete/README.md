# MSS(Musculoskeletal System)

## Abstract

This codes implement basic simulation and control of **Musculoskeletal** system. Skeletal movements are driven by the actuation of the muscles, which are coordinated by their activation levels. Interfacing with python and pytorch, it is available to use Deep Reinforcement Learning(DRL) algorithm such as Proximal Policy Optimization(PPO).

## How to install

### Install TinyXML, Eigen, OpenGL, assimp, Python3, etc...

```bash
sudo apt-get install libtinyxml-dev libeigen3-dev libxi-dev libxmu-dev freeglut3-dev libassimp-dev libpython3-dev python3-tk python3-numpy virtualenv ipython3 cmake-curses-gui
```

### Install boost with python3

We strongly recommand that you install boost libraries from the **source code**
(not apt-get, etc...).

- Download boost sources with the version over 1.66.(https://www.boost.org/users/history/version_1_66_0.html)

- Compile and Install the sources

```bash
cd /path/to/boost_1_xx/
./bootstrap.sh --with-python=python3
sudo ./b2 --with-python --with-filesystem --with-system --with-regex install
```

- Check yourself that the libraries are installed well in your directory `/usr/local/`. (or `/usr/`)

If installed successfully, you should have something like

Include

* `/usr/local/include/boost/`
* `/usr/local/include/boost/python/`
* `/usr/local/include/boost/python/numpy`

Lib 

* `/usr/local/lib/libboost_filesystem.so`
* `/usr/local/lib/libboost_python3.so`
* `/usr/local/lib/libboost_numpy3.so`


### Install DART 7.0

Please refer to http://dartsim.github.io/ (Install version 7.0)




### Install PIP things

You should first activate virtualenv.
```bash
virtualenv /path/to/venv --python=python3
source /path/to/venv/bin/activate
```
- pytorch(https://pytorch.org/)

```bash
pip3 install http://download.pytorch.org/whl/cpu/torch-0.4.1-cp35-cp35m-linux_x86_64.whl 
pip3 install torchvision
```

- numpy, matplotlib

```bash
pip3 install numpy matplotlib ipython
```

## How to compile and run

```bash
mkdir build
cd build
cmake ..
make -j8
```

- Run PPO
```bash
cd ../pymss
source /path/to/virtualenv/
python3 PPO.py
```
- Run UI
```bash
./render/render
```

- Run Trained data
```bash
./render/render ../nn/xxx.pt
```
