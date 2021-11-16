# gym-panda-manipulation

A panda manipulation for 3D Object Reconstruction. It aims that panda arm grasps and rotates an object in front of RGB-D camera, so that all aspects of the object can be recoded.


## Install

Conda virtual environment is recommended.

```
conda create -n [env_name] python=3.8
conda activate [env_name]
```

Example here:

```
conda create -n bulelt python=3.8
conda activate bullet
```

Install from source:

    git clone https://github.com/susu1210/panda_manipulation.git
    cd panda_manipulation
    pip install .

Other dependencies:

- pyreflexxes for trajectory planning: https://github.com/kschwan/pyreflexxes.git

Installation guide:

1. RMLTypeII

    ```
    git clone https://github.com/kschwan/RMLTypeII.git
    cd RMLTypeII
    cmake . -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
    make
    sudo make install
    ```

2. pybind11

    ```
    conda install -c conda-forge pybind11
    ```

3. pyreflexxes

    ```
    git clone https://github.com/kschwan/pyreflexxes.git
    cd pyreflexxes
    pip install --user .
    ```

- transformations

  ```
  pip install transformations==2020.1.1
  
  
  ```

  

## Basic Usage

updating
