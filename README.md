# SkelGen
This codes implement the algoirhtm of **Functionality-Driven Musculature Retargeting**. You can get musculoskeletal model with simple user input.



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


## How to compile and run

```bash
mkdir build
cd build
cmake ..
make -j8
./editor
```



## Parameteric Skeleton
1. Elongation and torsion of Femur, Tibia, Humerus, and Radius
2. Elongation and expansion of trunk
3. Elongation of neck

## Document
Project page : [Link](http://mrl.snu.ac.kr/research/ProjectFunctionalityDriven/fdmr.htm)

## Contributors
This work is done with

Hoseok Ryu(rhs0266.github.io)


## Reference
1. [Body height estimation based on tibia length in different stature groups](https://onlinelibrary.wiley.com/doi/full/10.1002/ajpa.10257)
2. [LENGTH OF LONG BONES AND THEIR PROPORTION TO BODY HEIGHT IN HINDUS](https://europepmc.org/backend/ptpmcrender.fcgi?accid=PMC1249729&blobtype=pdf)
3. [Muscles that move the arm](https://www.acefitness.org/fitness-certifications/resource-center/exam-preparation-blog/3535/muscles-that-move-the-arm)
3. [Muscles that move the leg](https://www.acefitness.org/fitness-certifications/resource-center/exam-preparation-blog/3594/muscles-that-move-the-leg)
