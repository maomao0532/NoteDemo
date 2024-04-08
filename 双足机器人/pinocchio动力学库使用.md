# Pinocchio动力学库使用

## 1. 安装

从源码安装：

```
git clone https://github.com/stack-of-tasks/pinocchio.git

cd pinocchio/

git checkout master

mkdir build

cd build/

git submodule update --init

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local

make -j8    

sudo make install
```

环境配置：

```
sudo gedit ~/.bashrc
添加
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```

## 2.常用功能

### 2.1 加载urdf构建模型

```c++
pinocchio::urdf::buildModel(urdf_filename, model);
pinocchio::Data data(model);
```

构建浮动基模型，需要加入浮动关节

```c++
pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), _model);
```

如此，构建的模型广义坐标由以下构成：浮动基位置xyz，浮动基姿态xyzw，关节角度q；

