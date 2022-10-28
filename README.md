# bldc-servo

This project turns a brushless motor and magnetic encoder into a servo with absolute position
control. It runs on the Mikroe [STSPIN233 Click board](https://www.mikroe.com/stspin233-click)
board. 

![STSTPIN233 Click](./doc/stspin233_click.jpg?raw=true)

It is controlled by serial commands sent at 115200, which can provide position targets, as well as 
torque commands, or initiate an encoder calibration sweep.

## Building

It's written with modm, so you'll need the modm python tools. 

`pip install modm`. 

Then you can build the libraries: 

`lbuild build` in the app directory.

Then you can compile / program:

`scons build`

`scons flash`

To program the STM32F0 on the board, you'll have to connect to the SWD header using an STLink or
equivalent SWD programmer.

## Caveats

This is a very quickly thrown together thing and not a ready product. Ignore or use as a reference/starting point only! 

Some issues: 

- It uses the analog output of the AS5600 encoder. This is not great; there's some hysteresis at the
  wrap-around, and I think the accuracy is not as good. Better to put it into PWM or I2C read modes. 
- It has no non volatile storage, and no automatic motor angle calibration. In order to commutate
  the motor, you have to do an offset calibration to relate encoder angle to motor electrical angle.
  Hypothetically it can calibrate itself on command and save the value to flash, but I have not
  implemented that. Instead it has to be calibrated by sending the "CAL" command, recording the 
  output data, and running the python script. Then the program must be re-compiled with the correct
  offset calibration. Look I warned you this was quickly thrown together. 



