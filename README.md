# Distributed_Systems
py -m venv .env
python3 -m venv .env

University Project
April 2023 - January 2024
Riccardo Tessarin and Simone Carli

## Project objectives
Creating a hardware infrastructure to test a distributed system algorithm. Using differential drive robots that move autonomously in space. 

Specifically, there is the implementation of a fully distributed communication network, exploiting ad Hoc protocol and the use of static ip address and the creation of a communication and messaging system for disseminating information among the agents in the network.

## Youtube demo
A demo of the code running is visible in:
https://youtu.be/A0rwAjhCfVU


### Used tools: 
- Batman-adv, a system for implementing ad hoc networks, which is already in the linux kernel;
- ZeroMQ a communication protocol that is fast and light to implement;
-  Arduino platform for controlling robots and reading sensors installed on the board
- Raspberry for communication, storage and wls algorithm execution
- Obstacle avoidance algorithms and control logic to make drones move
- wls algorithm both standard and a distributed

## Chapter subdivision
 1. Ad Hoc network implementation
 2. Arduino code
 3. Matlab code
 4. Python code

# 1. Mesh network implementation with Raspberry and Batman advance

To create the mesh network with raspberry the following steps were necessary:

1. Initially install 
***
```sudo apt-get update && sudo apt-get upgrade -y``` 
***

2. then run
***
```sudo apt-get install -y batctl``` 


You need to verify that Batman is loaded before trying to use it
enter this code in the file ```sudo nano /etc/modules```

```
# /etc/modules: kernel modules to load at boot time.
#
# This file contains the names of kernel modules that should be loaded
# at boot time, one per line. Lines beginning with "#" are ignored.

i2c-dev
batman-adv
```

3. Go to ```cd /etc/network/interfaces.d```

4. Create a file "sudo nano wlan0" and enter the following lines of code:
***
```
auto wlan0
iface wlan0 inet manual
    mtu 1468
    wireless-channel 1
    wireless-essid RPi-mesh
    wireless-mode ad-hoc
    # wireless-ap 02:12:34:56:78:9A
```

if a file **wlan0** is already existing, comment using # the already present code rows. 

It also works with the command ```iface wlan0 inet6 manual``` and setting to 1532 the value of MTU. 

5. Similarly, create a second file always in the same location, calling it **bat0**
***
```
auto bat0
#iface bat0 inet6 auto
#   pre-up /usr/sbin/batctl if add eth0
#   pre-up /usr/sbin/batctl if add wlan0
iface bat0 inet auto
   address 192.168.123.3
   netmask 255.255.255.0
   gateway 192.168.123.1
   pre-up /usr/sbin/batctl if add etho0
   pre-up /usr/sbin/batctl if add wlan0
```

The first three lines are commented out because in this configuration I am not using the protocol **ipv6**, but only the **ipv4**. 

### Mesh Node

6. For a mesh node, the following code should then be inserted within the file ```sudo nano /etc/rc.local ```

```
#!/bin/sh -e
sudo batctl if add wlan0 &
sudo ifconfig bat0 mtu 1468 &
sudo batctl gw_mode client &
sudo ifconfig bat0 up &
sudo ifconfig wlan0 up &

exit 0
```
7. Interrupt the DHCP process to prevent it from trying to manage the wireless lan interface using the following command :
***
```
echo 'denyinterfaces wlan0' | sudo tee --append /etc/dhcpcd.conf
```

At this point on reboot, the mesh node automatically connects to the created ad-hoc network. 

###  Gateway node
1. Install the following package for managing DHCP software
```sudo apt-get install -y dnsmasq```

2. Configure the DHCP server with the following settings:
```sudo nano /etc/dnsmasq.conf```
```
#To be inserted at the bottom of the entire annotated file
interface=bat0
dhcp-range=192.168.123.2,192.168.123.99,255.255.255.0,12h
```


3. For a Gateway node, the following code should then be inserted within the file ```sudo nano /etc/rc.local ```

```
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script  execute the sh file below


sudo batctl meshif bat0 if add wlan0 &
sudo ifconfig bat0 mtu 1468 &

sudo batctl gw_mode server &

sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE &
sudo iptables -A FORWARD -i wlan1 -o bat0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT &
sudo iptables -A FORWARD -i bat0 -o wlan1 -j ACCEPT &



sudo ifconfig wlan0 up &
sudo ifconfig bat0 up &
sudo ifconfig bat0 192.168.123.1/24 &



exit 0

```

## Assigning a static ipv4 address
1. Go to the following file: 
***
```sudo nano /etc/dhcpcd.conf```

2. Enter the command ```static ip_address=192.168.xxx.4/24 ``` 




##  ZeroMQ communication

Taking advantage of zeroMQ, it is possible to create the communication protocol and messaging app that will be used to carry out the experiment by sharing the status information of the various robots with each other.

TWO PUB-SUB type socket has been created: two programs run simultaneously in each machine: a first one acts as a server. It is a PUB and collects sensor readings, packages them and sends them to SUBs, which use these values with the WLS algorithm. 

Four servers and four subscribers are therefore needed. The subscriber is the same for all agents. The discovery (assignment of the ip addresses and tcp channel) is done manually, taking advantage of the static drone network setup done with Batman-adv.



### Serial communication
pip install pyserial

lsusb

dmesg | grep "tty"  e trova il nome della porta

### To check the connected devices
sudo arp-scan --interface=bat0 --localnet

### To check active process running on the raspberry
ps aux


This is the command  **sudo nano rc.local** and must be inserted in
```/usr/bin/python3 /home/pi/Desktop/example.py &amp; ``` 


# 2. Arduino
- There are several folder here that comprise both base command for use the Mbot and several test implemented to develop the final obstacle avoidance code. 
- folder "LAST_VERSION" has the code that communicate with the raspberry, perform the obstacle avoidance and is controlled by Rasperry. 

# 3. Matlab
- In "analisi_dati_Mbots" there are the code write to read the log of the Mbots and visuale, analyse the data
- In "Log_Mbots" there are the mBots data collected in some experiments
- In "wls_statiche" there are all the code realized to implement and test the wls program. Experiment with only one agent, multiple agents, latency, increasing erro in the distance measured. 

# 4. Python
Here there is the code implemented to perform the experiments with the real agents

## centrale_comando folder
The ojective is to control all the Mbots from a central deliver controller, in order to simplify the start, stop and reset of the Mbots operations.

- In **Command_central_Gateway.py** there is a PUB socket, with an INPUT from keyboard the program send at the Pi connected and waiting from instrunctions the action to perform:
Start, Stop or Reset for power up, terminate or reinitilize

## mbots_droni_p2p folder

- In **initial_script_3_Mbot.py** there is a script that must be executed when the Pi is launched: necessary to insert the instruction and execute at reboot. 
It attend for the command from the central station and can start the arduino wired connected, stop it and reset. Furthemore it starts the pythons code PUB and SUB to record the data of the experiment. 

- **uwb_routine.py** contains code to be inserted into each Mbot, this is done to
read the uwb in the background and save the values in a pickle file, which will then be read by subscriber. 
It simplify and make it more efficient the subscriber. In a subscriber you have to write a pickle file state when the code terminates, this code interrupts code execution.
- **subscriber_to_Mbot3.py** Here there are 4 SUB, to receive the information from all the nodes in the network and store in memory. It is the base code, the info collected can be used for algorithms
- **subscriber_real_solo_wls.py** read the data of uwb using "uwb_routine" and perform the localisation. Execution of the wls algorithm. 
- **wls_class.py** contains all the function required to perform the execution of the wls algorithm, both in simulation and with real agents. In a distributed or non distributed mode.
- **subscriber_to_Mbot_5_WLS_final.py** the developed code implementation to run with the real agents
- in **wls_mbot_2_macchine.py** and **wls_mbot_alone.py** there are the simulation perfromed with the python code 





# Use Procedure

## Setup Raspberry e Arduino
1. Load into raspberry that you use as a command center the scripts: 
- Command_central_Gateway.py
- global_variable_Gateway.py  
- central_pub_Gateway.py
- var_strings_Gateway_.py

2. Upload code to all Arduino Uno aka Makeblock Me Orion+

3. Load on all Raspberrys mounted in the MBots the scripts:

Use scp to copy the entire folder:

    scp -r [source_directory] [username]@[destination_host]:[destination_directory]

Per la 1:
```
        scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.77:/home/pi/Distributed
```
Per la 2:
```
        scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.9:/home/pi/Distributed
```
Per la 3:
```   
   scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.5:/home/pi/Distributed
```
Per la 4:
```
        scp -r /home/piktessa/Distributed/final_dec/mbots_droni_p2p/ pi@192.168.123.6:/home/pi/Distributed
```    

- initial_script_3_Mbot.py
- pub_to_sub_Mbot.py 
- subscriber_to_mbot_3_Mbot.py
- subscriber_to_Mbot_5_WLS_final.py
- var_strings_.py
- uwb_routine.pu


4. Find ip address, command **sudo arp-scan --interface=bat0 --localnet**. 
On **ALL** raspberry correctly assigns addresses and ports in **var_strings_.py** . 

5. On every Mbot lauch both initial_script_3_Mbot.py and subscriber_to_mbot_3_Mbot.py


6. launch command central e send comand calibrate, wait the time indicated, then digitate start

7. Acquisition of all the data

8. Copy the log in Central Raspberry using  **scp data_Mbot_00.txt piktessa@192.168.123.1:/home/piktessa/Distributed/log_Mbot_6**

**Per la 1:**

    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_1
    scp data_Mbot_12_18_2023__19_47_59.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_1
    scp logs/data_Mbot_rotation30.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_1
**Per la 2:**

    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_2
    
    scp logs/data_Mbot_rotation00.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_2

**Per la 3:**
    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_3
    scp logs/data_Mbot_rotation30.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_3

**Per la 4:**
    scp data_Mbot_100.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_4
    scp logs/data_Mbot_rotation00.txt piktessa@192.168.123.1:/home/piktessa/Distributed/num_4

    



