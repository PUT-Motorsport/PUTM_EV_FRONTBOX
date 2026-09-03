# PUTM_EV_Frontbox

PCB board aquiring data from the front side of the vehicle.

# Features

- APPS
  - read position of acceleration pedal and send to VCU
  - two outputs: power (+5V) lines with separate regulators
  - two inputs: APPS signal lines
  - check plasubility between two signal lines
- Data acquisition
  - front and rear brake pressure measurement
  - safety sensing
  - brake pedal position sensor for regenerative braking


### APPS

- 2 linear potentiometers mounted on pedal
- hardware filter and voltage clamp
- 2 diffrent transfer function to detect implasubility when defect occurs

### Front and rear brake pressure

- two analog, 5V sensors
- hardware filter and voltage clamp

### Safety (shutdown circuit) sensing
Optocupler to check state of SDC curcuit, shift level from 24V to 3V3 and read on MCU digital input
The safety state will be measured at:
- left kill switch
- right kill switch
- driver's kill switch
- inertia switch
- BSPD
- brake overtravel
- suspension interlocks

### Brake pedal position sensor
TBD

