## LIBMONGOCXX

The package requires **mongocxx** (C++ driver for mongoDB).
If mongocxx is not installed follow the instruction:
```
sudo apt install libbson-dev  libmongoc-dev
curl -OL https://github.com/mongodb/mongo-cxx-driver/archive/r3.1.4.tar.gz
tar -xvf r3.1.4.tar.gz
cd mongo-cxx-driver-r3.1.4/build
cmake . .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
sudo cmake --build . --target EP_mnmlstc_core
cmake --build .
sudo cmake --build . --target install
```

## MONGODB

Please install also [MongoDB](https://docs.mongodb.com/manual/tutorial/install-mongodb-on-ubuntu/) and [Compass](https://docs.mongodb.com/compass/current/install/)

**Tips**
If having troubles when installing Compass on Ubuntu 20.04, try:
```
sudo apt-get -f install
```
before the installation.