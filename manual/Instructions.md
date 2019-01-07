# FlyByLogic

### Requirements

1. IPOPT
2. casadi (With Ipopt and QPOASES)
3. MPT3 Toolbox
4. CVX

#### Reference https://github.com/casadi/casadi/wiki/SourceBuild

#### Compiling IPOPT from sources
On Linux and Mac OS X, IPOPT can be compiled from sources. You will find instructions on how to do this on their [website](https://www.coin-or.org/Ipopt). Our experience is that installing IPOPT is relatively easy if you follow the installation instructions on website carefully, but very hard if you try to improvise. Here is a condensed version of the installation instructions:
- Download the latest version from SVN:
 * `svn co https://projects.coin-or.org/svn/Ipopt/stable/3.11 CoinIpopt`
- Get third-party dependencies via the provided script:
 * `cd CoinIpopt/ThirdParty`
 * `cd Blas && ./get.Blas && cd ..`
 * `cd Lapack && ./get.Lapack && cd ..`
 * `cd Metis && ./get.Metis && cd ..` (Graph Coloring tool used by e.g. Mumps)
 * `cd Mumps && ./get.Mumps && cd ..` (Sparse direct linear solver with permissive license)
 * `cd ..; mkdir build; cd build`
- Compile and install IPOPT. Compiling `-fPIC` often helps:
 * `../configure --prefix=/usr/local ADD_FFLAGS=-fPIC ADD_CFLAGS=-fPIC ADD_CXXFLAGS=-fPIC`
 * `make; sudo make install`
 * If you wish to compile IPOPT with support for parametric sensitivities (sIPOPT), follow the instructions below.
- Test if `pkg-config` is able to find your Ipopt installation:
 * `pkg-config --libs ipopt`

If this last test does not return your installed Ipopt libraries, you must set the variable `PKG_CONFIG_PATH`, e.g. by adding `export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<your_ipopt_pkgconfig_path>` to your `.bashrc` or `.profile` file. `<your_ipopt_pkgconfig_path>` is the path where `ipopt.pc` of your Ipopt installation lives (typically `<ipopt_install_dir>/lib/pkgconfig`).

#### Adding additional linear solvers (MA27 Required)
To really benefit from from IPOPT, you should also try to get additional linear solvers, which can be done post-installation, regardless of how IPOPT was installed. We strongly recommend you to get at least the [HSL solver](http://www.hsl.rl.ac.uk/archive/index.html) MA27 (which is free). You will find instructions on that [here](Obtaining-HSL). When filling out the forms for obtaining HSL, please mention that you plan to use the routines with Ipopt/CasADi.

#### Building CasADi from sources

Check out CasADi from GitHub. We recommend you to check out the latest release, which is to be found in the `master` branch:
```
git clone https://github.com/casadi/casadi.git -b master casadi
```
After the initial clone, you can update with:
```
git pull
```
If you are on the `master` branch, this will give you the latest release. If you have checked out a release tag, any updates will be backwards compatible.
***


Create a build directory for an out-of-source build:
```
cd casadi
mkdir build
cd build
```

Generate a makefile and build
```
cmake -DWITH_IPOPT=ON -DWITH_QPOASES -DWITH_LAPACK ..
```

Look at the output and make sure that CMake was able to find the software that you wanted to use together with CasADi. If a certain software was not found correctly, edit the CMakeCache.txt file which should have been generated in your build directory. Note that CasADi will only compile interfaces to software it is able to locate.

Now build CasADi from source:
```
make
sudo make install
```


#### MPT3 Tool box
MPT 3.+ toolbox: Follow the installation instructions at http://people.ee.ethz.ch/~mpt/3/

#### CVX
Download the CVX contents from http://cvxr.com/cvx/download/. Follow the instructions listed in the website to setup CVX

## Setting up the C++ backend

Run the following commands in terminal (from the top level directory in this repository);

```
cd AATC_cpp/src/AATC
cmake .
make
./bin/FBL
```

## Mission planning via the MATLAB GUI
To perform custom missions, naviagate (in MATLAB) to the GUI folder and run Mission_GUI.m

See the instructions listed in GUI User Manual.pdf

####Note
Make sure that Casadi, CVX, MPT3 are in the right path.

Mission_GUI.m will invoke AATC_cpp by default to plan the mission. This can be changed to Matlab solver by XXXXXXXXXXX. However C++ solver is faster the Matlab solver by XXXX sec.
