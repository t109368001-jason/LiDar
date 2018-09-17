# LiDar

## Requirements
- VTK 7.1.0
- pcl 1.8.0

## Installation

```bash
sudo apt-get install -y build-essential cmake

git clone https://github.com/Xiao-Xian-Jie/LiDar.git

cd LiDar
mkdir build
cd build

cmake ..
make -j $(($(nproc) + 1))
./bin/pcl_viewer ../file/1534128682089346.pcd

#build all example
cmake -D BUILD_EXAMPLE=ON ..
make -j $(($(nproc) + 1))
./bin/pcd_viewer ../file/1534128682089346.pcd
./bin/pcd_poisson_viewer ../file/100.pcd
./bin/pcap_viewer ../file/123.pcap
```
