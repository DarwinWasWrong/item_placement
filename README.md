# item placement
item placement machine code

![image](https://user-images.githubusercontent.com/6262140/201211540-7c1bbd7a-d942-4a0e-aba6-f8ad45e395a9.png)


this is the original design



Eatitpal
OP
 — 02/11/2022
I'm building a automated parts feeder. 
I have the entire physical unit itself build at this point (Drivers, Step motors, Feeder itself.) All I need in the code itself!
I'm really hoping someone here will be able to help me through this.
I've tried to use a couple different people from fiver with no luck and I'm out a couple hundred bucks. 

I'm going to post two different videos and the code that was written by the fiver guys. (That doesn't work)

Video 1 - Explains how the unit will work.
Video 2 - Is the actual unit that we built.

If someone here can help or knows someone that can help it would be huge for me. Id be willing to compensate someone for their time once the feeder is working.

![image](https://user-images.githubusercontent.com/6262140/201211868-c03224a7-80fc-4e59-b44f-4bf3739e85ad.png)


https://drive.google.com/drive/folders/1tnN6S7c9GMuK3RCkOugg0v6PbQz7M-L-?usp=sharing
^^ this is a link to the 2 videos...
The explanation is pretty important.



- Arduino Uno R3: https://docs.arduino.cc/hardware/uno-rev3 

- Stepper motors Nema 23: https://www.omc-stepperonline.com/e-series-nema-23-bipolar-1-8deg-3-0-nm-425oz-in-4-2a-57x57x113mm-4-wires-23he45-4204s 

- Stepper drivers DM542: https://www.amazon.com/dp/B08XXKB36L/ref=twister_B07PSDXP8V?_encoding=UTF8&psc=1 

- Power supply 24V: https://www.amazon.com/Adjustable-DROK-110V-220V-Switching-Transformer/dp/B08GFSVHLS/ref=sr_1_4?crid=10WZ9XJAAFPBR&keywords=24v+power+supply&qid=1657310772&sprefix=24volt,aps,96&sr=8-4&th=1 



on start, we have to check for what condition the machine is in.

1. proximity switch must be HIGH - no items
2. SHELF LIMIT should be OFF
3. SHELF HOME  ON
3. TABLE BAR LIMIT should be off
4. TABLE BAR HOME should be ON
5. EMERGENCY SWITCH OFF 




Home Button - Sends vert motors down and Horizonal motors back until they hit switches to stop them.
Start button - Sends Horizonal motors forward periodically pausing when the prox switch is triggered
              finally completely finishing its cycle once a switch is triggered at the end of the cycle.



https://drive.google.com/file/d/1pfGOSFMlpMy-eEsUMzlPYLCXtyfiZos6/view?usp=sharing


![image](https://user-images.githubusercontent.com/6262140/201214316-00f1b181-aa46-48e0-b15e-6469dc070c17.png)
stepper drivers


![image](https://user-images.githubusercontent.com/6262140/201253898-47970235-4525-4e90-82b8-a53078c2a4dc.png)



Changes :


Replace Home button with: 

Button- raise shelf , stop at switch only 
Button- lower shelf , stop at switch only 
Button- table bar back, stop when button is released, max stroke at switch
Button- table bar forward, stop when bottom is released, max stroke at switch
Button Start- 
shelf must be lowered all the way to the switch. 
If shelf is up, then lower it in the program. 
If shelf is down, then just start.

 Also the table bar must be allowed to be positioned anywhere between the forward and back switch 

Auto Cycle- Table bar pushes parts forward, senses a part then stops, shelf up, stop 
When no part is sensed , shelf down, push forward till part sensed, stop, shelf up, stop When table pusher reaches forward switch and no more parts on table, stop. 
End of program.


NEW PANEL
![image](https://user-images.githubusercontent.com/6262140/202845173-b2097235-7f6d-4380-82c0-a9382b03ce0c.png)



ACTUAL Panel
![image](https://user-images.githubusercontent.com/6262140/202845253-0cfe31ee-c0c9-4087-8a5a-31dcd234573b.png)


Testing code for limits, homes, calibration and switches

https://wokwi.com/projects/348433529264144980


testing the motors were reversed. Changed the code to reverse the direction logic in the driver.

https://wokwi.com/projects/348433529264144980


E Series Nema 23 Bipolar 1.8deg 3.0 Nm(425oz.in) 4.2A 57x57x113mm 4 Wires

    Manufacturer Part Number: 23HE45-4204S
    Number of phase: 2
    Step Angle: 1.8 deg
    Holding Torque: 3.0 Nm(425oz.in)
    Rated Current/phase: 4.2 A
    Phase Resistance: 0.9 ohms± 10%
    Inductance: 3.8 mH ± 20%(1KHz)

Step Angle: 1.8 deg  200 steps per revolution
@ 1/2 stepping, that will give you 400 steps per revolution

So the physical driver change the switches to 400 steps from 10000/5000 steps





