# PUTM_EV_Frontbox_2023

PCB board aquiring data from the front side of the vehicle.

# Features

- APPS
  - two outputs: power (+5V) lines with separate regulators
  - two inputs: APPS signal lines
  - ability to disconnect the shutdown circuit
- Data acquisition
  - front and rear brake pressure measurement
  - safety sensing
  - imu measurements
  - steering wheel position sensor
  - brake pedal position sensor
  - suspension position sensors
- Laptimer
  - IR sensor

### APPS

Rotary potentiometers? Hall effect sensors?

### Front and rear brake pressure

Two analog, 5V sensors

### Safety (shutdown circuit) sensing

Where on the shutdown circuit will the board be placed?
Optimal placement is before BSPD (most straightforward connections)

The safety state will be measured at:
- left kill switch
- right kill switch
- driver's kill switch
- inertia switch
- BSPD (can be sensed directly from the neighboring board???)
- overtravel

### IMU measurements

ISM330DHCX mounted on the board (how will the mechanical requirements be fullfilled? reference axis, rigid mounting location?)

### Steering wheel position sensor

Sensor: AS5601

The sensor will be placed on a mini-board?

Will be powered from 5V and communicate with the main board over I2C

### Brake pedal position sensor

Same as APPS

### Suspension position sensors

Two linear potentiometers?

### IR sensor

What IC will be used?
IC:

