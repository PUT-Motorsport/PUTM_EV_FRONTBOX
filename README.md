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
<del>- Laptimer </del>

### APPS

Linear potentiometers:

### Front and rear brake pressure

Two analog, 5V sensors

### Safety (shutdown circuit) sensing

The safety state will be measured at:
- left kill switch
- right kill switch
- driver's kill switch
- inertia switch
- BSPD
- overtravel
- ASB

### IMU measurements

ISM330DHCX mounted on external board with swps:

### Steering wheel position sensor

Sensor: AS5600

The sensor will be placed on a mini-board. The angle signal uses PWM

### Brake pedal position sensor

Same as APPS

### Suspension position sensors

Two linear potentiometers

