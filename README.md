
This repository is a fork of this "mbrusa" project https://github.com/emmebrusa/TSDZ2-Smart-EBike-860C that was developed for TSDZ2 Tongsheng motor when used with a 860C display.

The purpose here is to have a version that can be used on a TSDZ8 Tongsheng motor.


Note: this version requires that the 860C display is reprogrammed with a special firmware. This firmware has also been developped by mbrusa for the TSDZ2. This TSDZ8 version uses exactly the same firmware on the 860C display. So, to know how to install and use it, please look at the mbrusa project mentionned here above. 


Changes of the controller firmware were required because TSDZ8 uses a different microprocessor (XMC1302).

This version is supposed to provide the same functionalities as the mbrusa TSDZ2 project above.


Compare to the TSDZ8 original firmware there are some expected benefits:
* The user can adapt many parameters to his preferences.
* The display can show more data (without having to change the display firmware) and allow you to make a few changes to the setup.

For more information on the TSDZ2 OSF version, you can: 
* look at Endless Sphere forum reference thread: [endless-sphere.com.](https://endless-sphere.com/forums/viewtopic.php?f=30&t=110682).
* see the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-860C/wiki) from mbrusa

IMPORTANT : at this stage, this is just a beta version. It has NOT been tested on a bike and there are probably some bugs.
Try it at your own risk!!!!


To use this firmware, you will have to:

* Donwload this firmware
* Use a Segger Jlink device and a cable (this device replace the Stlink used for TSDZ2)
* Flash the compiled firmware on the TSDZ8 controller
* Update the firmware running on the 860C display and fill your parameters on the display

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

I cut the cable and connect it based on the diagram given in "DOC" folder in "Diagram-TS 32-bit Mid Drive Motor Controller Programming Cable (EN).pdf". It is safe to check that the extension cable you get uses the same colors.

Note: After cutting the extension cord in 2 parts, I used the one with the female connector and connected it directly to the motor.
I expect (not tested yet) that it is also possible to use the part with the male connector. The advantage is that you can keep the speed sensor cable connected to the motor when you flash. You connect then the Jlink device cable to the second yellow connector present on the speed sensor cable (instead of the cable foreseen for the lights).  

# 3.Flash the firmware

To flash, you can follow the instructions from here to know how to use Jlink: 
https://ebikestuff.eu/en/content/18-tsdz8-firmware-update-using-j-link-jlink-v9-programming-kit

The HEX file to upload is the one you downloaded from this github site at step 1.
It is named OSF_TSDZ8_860C_Vxx_xx.hex where xx.xx is a version number.

Note: while flashing, the motor should not be powered by the battery. Disconnect it or at least power it OFF. In principe, the Jkink will provide power to the controller (at least if it is a Jlink clone device).
It seems possible to keep the motor connected the display but do not press any button on the display.

# 4.Update the firmware running on the 860C display and fill your parameters on the display

See the instructions on the mbrusa site (see links above)

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