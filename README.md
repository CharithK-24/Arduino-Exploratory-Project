# Arduino-Exploratory-Project
## Lock system using Hand signs 

This project is inspired by the TV show/anime called “Naruto” . In the anime, hand
seals are designed to aid people by properly summoning and molding chakra (life
force) necessary to perform a technique. There are different sequences of hand
seals for every technique. Tried to implement the idea in real life; in short if the
sequence of the hand signs is correct, it will unlock the solenoid lock.


## 1. Wiring The Transmitter (Glove) :
 
 It consists of Arduino NANO, 9V battery, MPU6050, Flex sensor and NRF24L01 +2.4GHz Wireless RF transceiver module.
 The microcontroller i.e., NANO collects all the information from the sensing unit (Flex Value & IMU) and send signal to
 the receiving circuit via NRF24L01. The MPU6050 consists of 3-axis accelerometer and 3-axis gyroscope, which helps us to 
 measure acceleration, velocity, orientation and other motion related parameters.

## 2. Wiring The Receiver (The Safe) :

It consists of Arduino UNO, 9V battery, Relay module, Solenoid door lock and NRF24L01 +2.4GHz Wireless RF transceiver module.
The microcontroller i.e., UNO is used to control the solenoid lock, the solenoid lock requires higher current than the arduino UNO can
provide and to drive the solenoid lock we would need an external power source that can give 12V 500mA and for that we need a relay module 
which is an electromagnetic switch operated by a relatively small current that can control much larger current like solenoid door lock.


### Testings :

    #1 Issue: Range for the flex sensor (Flex Value)

    Had to set a flexible potentiometer range which is nothing but the resistance directly proportional to the amount of bending. 
    Thereby, tested the flex sensor individually and noted down the flex values multiple times for every hand sign required.

    #2 Issue: Filtering IMU noise from MPU6050
	

    MPU6050 is basically the combination of gyroscopes and accelerometers referred to as an Inertial Measurement Unit or IMU. This sensor can detect angle changes. 
    The angle readings from an IMU carry much noise. An IMU occasionally presents values that are 
    incoherent and vary unreasonably to one another. To get proper and clear values, a filter is needed. 

    So had to install a filter i.e Kalman filter library, an algorithm which uses a series of measurements observed over time, 
    in this context an accelerometer and a gyroscope. The Kalman filter will then try to estimate the state of the system, based on the current and previous states, 
    that tend to be more precise than the measurements alone.

	  #3 Issue: Communication between the transmitter and receiver (ARDUINO NANO AND UNO)

     Had to figure out the addresses(memory map) of the NANO and UNO ports in order for communication to happen i.e, 
     for the transmitter and receiver arduino codesto communicate through the NRF24 radio.


## Schematics:

![transmitter_wiring_diagram_DuWMTbnRVY](https://user-images.githubusercontent.com/70831607/215320825-c7a3daee-e394-458e-bde5-e650afe01aec.jpg)
![receiver_wiring_diagram_Z0wAicaB9J](https://user-images.githubusercontent.com/70831607/215320846-11c952b3-1f5b-41eb-b420-9b4208c3b04b.jpg)

