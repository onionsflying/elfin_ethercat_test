elfin_ethercat_test
======
Please install [elfin_robot](https://github.com/hans-robot/elfin_robot) first because this package depends on it.

---
### ATI force torque sensor  
#### Parameters in ati_fts_test.launch
* elfin_ethernet_name: name of the network interface card
* slave_number: slave number of the ATI F/T sensor

#### Test the ATI F/T sensor
```sh
$ sudo chrt 10 bash
$ roslaunch elfin_ethercat_test ati_fts_test.launch 
```