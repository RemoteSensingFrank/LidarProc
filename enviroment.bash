echo "install cmake"
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
wget https://cmake.org/files/v3.12/cmake-3.12.2-Linux-x86_64.tar.gz
sudo tar zxvf cmake-3.12.2-Linux-x86_64.tar.gz
sudo mv cmake-3.12.2-Linux-x86_64 /opt/cmake-3.12.2
sudo ln -sf /opt/cmake-3.12.2/bin/*  /usr/bin/

#boost library
echo "install boost library:"  
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
sudo apt-get install libboost-all-dev
endTime=$(date +"%Y-%m-%d %T")
echo $endTime

#opencv instsall
echo "install opencv library"  
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
sudo apt-get install libopencv-dev
endTime=$(date +"%Y-%m-%d %T")
echo $endTime

#gdal install
echo "install gdal library"  
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
sudo apt-get install libgdal-dev
endTime=$(date +"%Y-%m-%d %T")
echo $endTime


#gtest install
echo "install gtest library"  
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
sudo apt-get install libgtest-dev
cd /usr/src/gtest
sudo mkdir build
cd build
sudo cmake ../
sudo make
sudo cp libgtest*.a /usr/local/lib
endTime=$(date +"%Y-%m-%d %T")
echo $endTime

#download and install liblas
echo "install liblas library"  
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
cd /home
sudo git clone https://github.com/libLAS/libLAS.git
sudo mkdir buildliblas
cd buildliblas
sudo cmake ../libLAS/
sudo make
sudo make install
endTime=$(date +"%Y-%m-%d %T")
echo $endTime

#download and instsall laszip
echo "install laszip library"  
stTime=$(date +"%Y-%m-%d %T")
echo $stTime
cd /home
sudo git clone https://github.com/LASzip/LASzip.git
sudo mkdir buildlaszip
cd buildlaszip
sudo cmake ../LASzip/
sudo make
sudo make install
endTime=$(date +"%Y-%m-%d %T")
echo $endTime

#PCL
echo "install PCL library"  
stTime=$(date +"%Y-%m-%d %T")
cd /home
echo $stTime
sudo apt-get install libflann-dev
sudo apt-get install libgl1-mesa-dev
sudo apt-get install libcap-dev
sudo apt-get install libpcap0.8-dev 
sudo apt-get install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
sudo ln -s /usr/include/eigen3/unsupported/ /usr/include/unsupported
sudo git clone https://github.com/PointCloudLibrary/pcl pcl-trunk
cd pcl-trunk 
sudo mkdir buildpcl && cd buildpcl
sudo cmake -DCMAKE_BUILD_TYPE=Release ../
sudo make 
sudo make install
cd home
endTime=$(date +"%Y-%m-%d %T")
echo $endTime
