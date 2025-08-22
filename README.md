
This repository is a fork of this "mbrusa" project https://github.com/emmebrusa/TSDZ2-Smart-EBike-860C that was developed for TSDZ2 Tongsheng motor when used with a 860C or SW102 display.

The purpose here is to have a version that can be used on a TSDZ8 Tongsheng motor.


Note: this version requires that the 860C or SW102 display is reprogrammed with a special firmware. This firmware has also been developped by mbrusa for the TSDZ2. This TSDZ8 version uses exactly the same firmware as TSDZ2 on the 860C or SW102 display. So, to know how to install the firmware on 860c or SW102 and how to use it , please look at the mbrusa project mentionned here above. Please note that the firmware to upload on the 860C is different from the SW102.


For SW102, the firmware to be used is here: https://github.com/emmebrusa/Color_LCD_860C/blob/master/releases/v20.1C.5-860C/sw102-otaupdate-20.1C.5.zip


For 860C, the files to be used are here : https://github.com/emmebrusa/Color_LCD_860C/tree/master/releases/20.1C.5-1-860C


Changes of the controller firmware were required because TSDZ8 uses a different microprocessor (XMC1302).

This version is supposed to provide the same functionalities as the mbrusa TSDZ2 project above.


Compare to the TSDZ8 original firmware there are some expected benefits:
* The user can adapt many parameters to his preferences.
* The display can show more data (without having to change the display firmware) and allow you to make a few changes to the setup.

For more information on the TSDZ2 OSF version, you can: 
* look at Endless Sphere forum reference thread: [endless-sphere.com.](https://endless-sphere.com/forums/viewtopic.php?f=30&t=110682).
* see the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-860C/wiki) from mbrusa

IMPORTANT : at this stage, this is just a beta version. There are probably some bugs.
Try it at your own risk!!!!


[![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/donate/?hosted_button_id=AD6HTLE53YDXW)


To use this firmware, you will have to:

* Donwload this firmware
* Use a Segger Jlink device and a cable (this device replace the Stlink used for TSDZ2)
* Flash the compiled firmware on the TSDZ8 controller
* Update the firmware running on the 860C display
* Fill your parameters using the display

If you have questions on this Tsdz8 project, you can ask on this forum:
https://endless-sphere.com/sphere/threads/new-tsdz8-pswpower.120798/page-12

If you find bugs, best is to open an issue in github : https://github.com/mstrens/OSF_860C


# 1.Download this firmware

Download or clone this repository. 
If you downloaded, unzip the archive where you want.

# 2.Preparing Jlink

You need a Segger Jlink device and a cable.
You can get it from here : https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

This link provides also a link to an archive with all the files for flashing. You can download and unzip it.

Note: I tested with a chinese jlink clone V9 from aliexpress and it worked too.

For the cable, you can also make your own cable with a speed sensor extension cord for TSDZ2 like this:https://fr.aliexpress.com/item/1005007479961045.html?spm=a2g0o.order_list.order_list_main.120.21ef5e5bFWfkqS&gatewayAdapt=glo2fra

I cut the extension cable and connect it directly to the Jlink flat cable based on:
- the diagram of Jlink connector : https://www.segger.com/products/debug-probes/j-link/technology/interface-description/
- the picture of speed cable connector : https://empoweredpeople.co.uk/2020/05/28/tongsheng-tsdz2-what-firmware-options-are-there/
In my case (be careful the colors may be different)
- red = SWDIO = TMS = pin 7 of Jlink
- black = SWCLK = TCK = pin 9 of Jlink
- brown = Vcc = VTRef = pin 1 of Jlink
- orange = Grnd = e.g. pin 4 of Jlink

It can also be useful to look at this link:  https://www.facebook.com/photo?fbid=7877850202251083&set=pcb.430249463263185.
It is safe to check that the extension cable you get uses the same colors for the same pins on the connector because some cables could use different colors/pin out.

Tip: After cutting the extension cord in 2 parts, I used the one with the female connector to connect to the motor (and to Jlink).
I also reconnected the wires from the part with the male connector. The advantage is that I can also connect the male connector to the speed sensor.  This avoid having to connect/disconnect the cable each time you flash the controller during the setup phase.


IMPORTANT POINT : 
If the controller is powered by the battery AND simultaneously by the Jlink device, there is a huge risk that it would be DAMAGED and unusable. Torque sensor driver could be destroyed. You would have to replace a mosfet transistor named Q7 on the controller board. Sill replacing it is not easy and require some skill.


By default Chinese clone Jlink (as the one provided by ebikestuff) usualy provides 3.3V on the Vref pin.
So take care to never power the motor with the battery when Jlink is connected.

For safety, I would recommend to avoid that Jlink provides the 3.3V. This can be achieved in 2 ways:
- if you make your own cable, just do not connect Vref from jlink to the cable
- if you use a cable where Vref is connected, you can (as far I tested) open the case of chinese Jlink device and remove a jumper. Then check with a voltmeter that the device does not provide anymore the 3.3V

Note : original Segger jlink devices does not provide 3.3V on Vref pin by default. Still it can provide the 3.3V if you send some command (see Segger doc if you want to enable it).


# 3.Flash the firmware

To flash, you can follow the instructions from here to know how to use Jlink: 
https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

The HEX file to upload is the one you downloaded from this github site at step 1.
It is named OSF_TSDZ8_860C_Vxx_xx_xx.hex where xx_xx_xx is a version number.

IMPORTANT NOTE: as said before, for flashing, the motor should not be powered by the battery AND simulterneously by the jlink device.
So you must OR disconnect the battery (or at least power it OFF) OR take care that your Jkink does not provide power on Vref pin to the controller.


# 4.Update the firmware running on the 860C display 

See the instructions on the mbrusa site (see links above)

# 5.Fill your parameters on the display

See the instructions on the mbrusa site (see links above).
Still there are a few differences:
* Coast brake ADC : This concept is not used in OSF TSDZ8. Still this field has been "reused" in order to let the user specify the "FOC multiplier" that is used to calculate "FOC angle" that plays a role in motor efficiency and stability when current is high. So enter here a value for "FOC multiplier". It seems that a value in the range 24/28 is good. Feel free to experiment but be careful with values that would be very different and could create unstability and let the motor become too hot.


* calibration MUST be disabled. If you enable it, 860C transmit some false data to the controller.


* Torque ADC step adv is not used (as calibration should be disabled)


* Torque offset adj is not used.


* Torque range adj is used to increase/decrease sensitivity for low pressure on the pedal. Sorry if the name if confusing but it was the only field from 860C that I could reuse for this. This parameter does not change the maximum assistance provided for any selected level when pressure on pedal is maximum but it allows to increase (or decrease) the assistance when pressure on pedal is quite low.
This parameter can vary between 1 and 40. When this parameter is set on 20, the assistance is calculated based on the value of the torque sensor.
The more the parameter is higher than 20 (up to 40) the less assistance you will get for small pressure on the pedal (but so the more you get for highier pressure - still never exceeding the max value defined for the selected level). In other words, the ratio assistance per kg pressure is lower for lowest pressure and higher for highest pressure compared to parameter set on 20.
Reversely, the more the parameter is lower than 20 (up to 1), the more assistance you will get for small pressure on the pedal. In other words, the ratio assistance per kg pressure is higher for lowest pressure and lower for highest pressure compare to parameter set on 20.


* Torque angle adj is not used (860c value is discarded)


* Torque ADC offset is very important. In TSDZ2 or in previous versions, the firmware read the torque sensor during the first 3 seconds and considers this value as the reference when no load is applied. This was done in order to get an automatic recalibration at each power on. This process is not good for TSDZ8 because, for some TSDZ8, the value varies significantly with the position of the pedal. So in this version of OSF, there is no autocalibration of the torque sensor with no load at power on. Instead, the user has to fill in "Torque ADC offset" the value that will become the reference. To find the value to encode, you must use the menu "Technical" and look at the field "ADC torque sensor". When no load is applied on the pedal, turn manually the pedal and look at the values in "ADC torque sensor". Note the MAXIMUM. I expect that value should be between 150 and 190 depending on your motor. Then add some margin (e.g 10) to avoid assistance with very low pressure and enter the value in "Torque ADC offset" (in "Torque sensor menu"). You can adapt the "margin" value to your preference (a higher value will require more pressure on the pedal before getting assistance but will reduce consumption).


* Torque ADC max has to be filled : to find the value, go to technical menu, look at field ADC torque sensor when you apply the max pressure on the pedal (about 80 kg = full human weight) while holding the brakes. It seems that the value should be around 450 for TSDZ8.


* Torque ADC step is important because it is used to convert raw values provided by the torque sensor into the human torque and so also to calculate the human power and the requested motor power. Measures done on only one TSDZ8 motor shows that the raw values provided by the torque sensor are proportional to the weight on the pedal up to about 50Kg (above there is a saturation) and that the raw value at 50kg seems to be about 85 % of the total range (difference between max and offset). In TSDZ8 firmware, ADC values are normalised in order to have a range of 160 steps. So Torque ADC step could be estimated with : 50kg *167 / (85% * 160) = 61. Note: it could be that this value is defferent for your motor.


* Values for the different assist levels/modes: The max value that can be filled with the 860C is usually 254. Still TSDZ8 can provide more power than TSDZ2. In order to get access to the full power even for lower weight on the pedal the ratio value/assitance has been changed for TSDZ8. You have to use a lower value (2 X lower) to get the same assistance for Power, Torque and Hybrid assist modes.


* If you use throttle, you have to set up the limits (min and max) provided by your sensor. In menu "Motor temperature" you have to declare that sensor is used for throttle and you have to fill the min and max values. To know them you have to use menu "Technical" and look at ADC throttle field. Note the min and max values when pressing throttle. For min, it is safe to enter a value that is slightly higher that what you read (to avoid unexpected start of the motor)

# IMPORTANT NOTES
* Installing this firmware will void your warranty of the TSDZ8 mid drive.
* We are not responsible for any personal injuries or accidents caused by use of this firmware.
* There is no guarantee of safety using this firmware, please use it at your own risk.
* We advise you to consult the laws of your country and tailor the motor configuration accordingly.
* Please be aware of your surroundings and maintain a safe riding style.


# Developper

If you want, you can look at the software and modify it.

This software has been developped with 
 - Modus toolbox (from infineon)
 - Visual Studio Code (and some extensions).
Note: Segger Jlink is also used to flash the controller if you want to do it inside VS Code.

To install those firmwares, you have to follow the instructions provided in this link in the steps 1.1 and 1.2 (and 1.3 if you plan to use Jlink inside VS Code)
https://www.infineon.com/dgdl/Infineon-Visual-Studio-Code-user-guide-UserManual-v04_00-EN.pdf?fileId=8ac78c8c92416ca50192787be52923b2&redirId=248223

This can be quite long but is not very difficult.
There is no need to follow the instructions in chapters 2 and after.

Note: on my side I still had an issue to use Jlink to directly flash/debug the firmware and I had to
- rename the Jlink folder to remove the version nr (so it becomes just "SEEGER")
- edit the .vscode/settings.json file in this project to adapt the paths to Segger tools

Still take care to put all the folders inside an empty folder in the case you would like to develop/compile your self. The m 

Then there are some more steps to perform:
- open "modus-shell"  (this tool is part of the programs that have been dowloaded with modus toolbox)
- in modus-shell, make the folder named "OSF" the current folder (use "cd" commands)
- then enter in modus-shell the command "make getlibs" ; this will copy several libs
- then enter in modus shell the command "make vscode" ; this will generate/update some files in .vscode folder

Normally you are now ready to use VS Code and to compile.

To compiling the firmware:
* Open VS Code.
* In menu "File", select the option "Open Workspace from File..."
* Select a file named "TSDZ8.code-workspace" in folder "OSF".This should open all files.
* Select the menu "Terminal" and the option "Run Build Task"

Check if errors are generated. It is not abnormal to get quite many "Warnings" (but not "errors").
This step created a HEX file that can be flashed in the controller.
 
Note : it is also possible to flash the code directly from VS Code at the same time as you compile (e.g. to debug it) but this requires probably to modify some set up in .vscode/settings.json file in order to let VS Code knows the location of some Segger files (depend on the folder where you installed Segger Jlink).

Note: if you compile the firmware yourself, the generated hex file to upload is named "mtb-example-xmc-gpio-toggle.hex" and is present in
folder "test_gpio_in/build/tsdz8_for_GPIO_TEST/Debug" or in "test_gpio_in/build/last_config" (depending if you use the menu Run/start debugging or the menu Terminal/Run build task)