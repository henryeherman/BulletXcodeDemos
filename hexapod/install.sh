# Pyzmq for zeromq python bindings 
pip install --upgrade pyzmq

# message pack for c++ and python
easy_install msgpack-python
brew install msgpack

# Pybrain Installation

    # Get Numpy and Scipy, See directions at 
    # http://www.scipy.org/Installing_SciPy/Mac_OS_X

    # Get numpy
    git clone https://github.com/numpy/numpy.git
    cd numpy
    python setup.py build
    sudo python setup.py install 
    
    # Get Fortran compiler from http://r.research.att.com/tools/gcc-42-5666.3-darwin11.pkg 
    # If Mac with Xcode 4.2 and install
    # Otherwise see website above
    
    # Get Scipy
    git clone https://github.com/scipy/scipy.git
    cd scipy
    python setup.py build
    sudo python setup.py install

    # Finally, get Pybrain
    easy_install pybrain

