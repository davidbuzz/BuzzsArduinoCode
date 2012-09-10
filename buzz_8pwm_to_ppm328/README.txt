# buzz's 8 channel PWM to PPM converter for RC use. 

# start with:
* an Arduino with a Atmega328 or 328p chip on it. 
* IDE to program the chip with this code.

How:?
* Used Arduino IDE to program this firmware onto the Arduino chip.
* Connect upto 8 RC PWM input signals so that the wires go to:
     red = 5v
     black = GND or 0V pin on arduino
     white = PWM signal pins, these connect to D0,D1,D2,D3,D4,D5,D6,D7

* Connect the PPM output so that the wires go to:
     red = 5v
     black = GND or 0V
     PPM out = D10 

Done! 

TIPS:  
* any channel that you don't connect a PWM input on, will emit a default value ( which is 1500 for all channels excpt throttle, which is 1100.
* disconnecting any channel after booting will cause the system to use the last-known-position of the input, until a good signal returns.
* D0 and D1 are also used by the Serial Programmer/bootloader.   You can not reprogram this unit with PWM inputs still connected to D0 and D1, unplug unit from PWM sources before re-programming.
* THis is not a "failsafe" unit, if has no failsafe functionality.
* If this code causes your expensive toys to crash, or worse, it's your fault for being a bad person, so you should be nicer, as karma's a bitch. 
* Not my fault.  none of it.   I love you too, now go away.
