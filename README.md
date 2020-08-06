# demo2d

This is a set of C++ tools for designing 2D demos. It is based on opencv.


vq3 is a result of the <a href="http://interreg-grone.eu">GRONE project</a>, supported by the Interreg "Grande Région" program of the European Union's European Regional Development Fund.

# Unix Installation

First, get the files.

``` 
git clone https://github.com/HerveFrezza-Buet/demo2d
``` 

Then, you can install the package as follows.

```
mkdir -p demo2d/build
cd demo2d/build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
sudo make install
cd ../..
```

Warning : If cmake complains about opencv, you may have opencv3 installed, rather than opencv4. In this case, you can checkout the opencv3 tag, and build. This is what we describe hereafter.

```
mkdir -p demo2d/build
cd demo2d/build
git ckeckout opencv3
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
sudo make install
cd ../..
```



The documentation is in /usr/share/doc/demo2d
