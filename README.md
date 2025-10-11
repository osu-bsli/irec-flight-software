# Ubuntu dependencies

```
sudo apt install -y build-essential cmake ninja-build pkg-config autoconf autoconf-archive libtool
```

# How to get UART working on Pi
Go into `/boot/firmware/config.txt` and remove `console=serial0,115200` if present. This will free up `/dev/ttyS0` for applications to use.
