# Arduino-Exploratory-Project
Lock system using Hand signs 

This project is inspired by the TV show/anime called “Naruto” . In the anime, hand
seals are designed to aid people by properly summoning and molding chakra (life
force) necessary to perform a technique. There are different sequences of hand
seals for every technique. Tried to implement the idea in real life; in short if the
sequence of the hand signs is correct, it will unlock the solenoid lock.

The language of this product consists of 12
hand signs to which the user can freely set any 5 hand signs in any sequence
according to the user’s choice.

## 1. Wiring The Transmitter (Glove) :
It consists of Arduino nano, 9 V battery, MPU 6050, Flex sensor, and NRF24L01+
2.4GHz Wireless RF Transceiver Module. The Microcontroller used for the
transmitter was arduino nano. The arduino nano will collect all information from
the sensing unit(Flex sensor and MPU6050) and send a signal to the receiving
circuit through the NRF24L01. The 9V battery was used to power arduino nano.
We use the Vin and Ground pin. A regulator exists on the Arduino board to reduce
the 9V supply to a steady 5V. The MPU6050 consists of a 3-axis Accelerometer and
3-axis Gyroscope inside it. This sensor helps us to measure acceleration, velocity,
orientation, displacement and many other motion related parameters of a system
or object. This chip uses I2C (inter-integrated circuit) protocol for communication.
A flex sensor is basically a variable resistor that varies in resistance upon bending.
Since the resistance is directly proportional to the amount of bending, it is often
called a flexible Potentiometer. We can measure that change using one of the
Arduino’s analog pins, but to do that we need a fixed resistor (not changing) that
we can use for that comparison ( using a 10K resistor). This is called a voltage
divider and divides the 5V between the flex sensor and the resistor.
NRF24L01 2.4 GHz transceiver module uses the 2.4 GHz band and it can operate
with baud rates from 250 kbps up to 2 Mbps and it can be used for wireless
communications at up to 100 meters.The operating voltage of the module is from
1.9 to 3.6V.

## 2. Wiring The Receiver (The Safe)
It consists of Arduino uno, 9 V battery, relay module, solenoid door lock, LED, and
NRF24L01+ 2.4GHz Wireless RF Transceiver Module. We use Arduino uno to
control the solenoid door lock. Solenoid door lock is electromagnetic lock, it is
made of a big coil of copper wire with an armature (a slug of metal) in the middle.
When the coil is energized, the slug is pulled into the center of the coil. Making
this solenoid able to pull from one end. The solenoid door lock requires higher
current than the arduino can provide,to drive the solenoid door lock we would
need a power source that can give 12V, 500mA and the relay module will be
driving it. A relay module is an electromagnetic switch operated by a relatively
small current that can control much larger current like solenoid door lock.

## 3. Transmitter/Receiver code :
The Arduino Nano - MPU6050 consists of a 3-axis Accelerometer and 3-axis
Gyroscope inside it. This combination of gyroscopes and accelerometers is
commonly referred to as an Inertial Measurement Unit or IMU. This sensor can
detect angle changes. The angle readings from an IMU carry much noise. An IMU
occasionally presents values that are incoherent and vary unreasonably to one
another. To get proper values, clear values, a filter is needed.
We then upload the codes to Arduino Nano (Transmitter) , Arduino Uno (Receiver)
through USB cable from the PC.
