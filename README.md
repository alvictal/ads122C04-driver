# AD122C04 Linux driver

## Driver Info

- Version: 0.1.0

## Installing the driver

In order to build this driver you must have a prebuilt kernel available that
contains the configuration and header files used on the target.
Also, the kernel must have been built with IIO class and modules enabled.

## Installing out-of-tree

The steps to deploy the driver on the target platform are:

### compiling

- You can either compile on the target system or you can cross-compile on a host machine
- If you cross-compile you must set the variable CROSS_COMPILE use `make KSRC={source_to_your_module} modules`

### installing

You can install the driver can be either installed directly on the host or on a remote target

- If you want to install it on a remote target use `make deploy` with the variable TARGET_IP set to the IP of the target machine in order to copy the files necessary for the instalation
use the script install.sh to install the driver on the working device

## Installing in-tree

The steps to deploy the driver on the target platform are:

- put the driver in `linux/drivers/iio/adc`
- modify the Kbuild and Makefile from `linux/drivers/iio/adc`
- select as module in `make menuconfig`
- install the modules in a different directory
- On your target machine replace the `/lib` directory with the one obtained   at make modules_install

## Device-Tree bindings

The driver can use device-tree bindings in order to optain the configuration parameters for each device. The "ti,ads122c04.yaml" file describes the device-tree bindings required by this driver.

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License 

This project is licensed under the MIT License - see the LICENSE file for details.