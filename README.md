# CC3D-PPM-Joystick
Converts the CC3D board into a USB joystick, uses PPM from receiver

This is an alternative firmware for a CC3D board, that allows the board to function as a PC Joystick. Simply plug the CC3D into a computer's USB port and plug a PPM receiver into the board's **CH1** input, and you have a working wireless joystick.

# Flashing the firmware
In order to use this firmware, you will need to flash it into the CC3D board's STM32F103 chip. For this, you will need:
- USB-to-serial dongle (also called FTDI) working with 3.3V logic levels and with a +5V output pin
- JST-XH 4-pin connector (for CC3D's main port, http://opwiki.readthedocs.io/en/latest/user_manual/cc3d/cc3d.html#connection-details ) that can connect to the FTDI dongle
- STM's firmware flash tool: UM0462, http://www.st.com/en/development-tools/flasher-stm32.html
- (possibly) a few wires, ideally a breadboard, to make the connections
- blade, prong or other conductive object to bridge the CC3D boot pads

NOTE: SouthQuay3D has a tutorial with pictures detailing a very similar procedure: http://www.southquay3d.com/index.php?route=news/article&news_id=9 . Some of the information there is missing and some is not necessary, though.

Orient the CC3D board so that the F103 chip is facing you, the receiver port is to the left, the output port is to the right, and main poirt and flexi port are downwards. Now the main port's pins are, left-to-right: GND, Power, TX, RX ( http://opwiki.readthedocs.io/en/latest/user_manual/cc3d/cc3d.html#cables-colors-pin-outs ). Connect the GND, TX and RX pins to your FTDI dongle (TX to dongle's RX, RX to dongle's TX), but do not connect Power just yet. Locate the CC3D boot pads - two pads in the top right corner of the board, usually labeled SBL and +3.3V. Using a knife or similar, bridge the two pads and connect the Power pin to your FTDI's +5V pin. If the board blinks its Status LED, you didn't bridge the boot pads properly, unplug the Power and try again. The result should be a board with only the Power LED lit, you can un-bridge the pads at this point. Bridging this way is rather difficult to get right, it might be easier to simply solder the two pads together; they can stay soldered while you flash the firmware; then un-solder once the firmware has been flashed.

Next, fire up the STM firmware flasher tool. In the first page, choose the COM port of your FTDI dongle, leave the other settings as they are (baudrate 115200, parity Even, echo Disabled) and click Next. Immediately the second page of the tool should come up, if not, you didn't bridge the boot pads properly, close the flasher tool, unplug Power and retry. On the flasher's second page, check that it says "target is writable" and "flash size 128 KB", then click Next. On the third page, choose "STM32F1_Med-density_128K" from the dropdown and click Next.

Finally the interesting fourth page. Select the "Download to device" radio button, then click the triple dots next to the file name field. Browse to the CC3D-PPM-Joystick firmware file (.hex or .bin, doesn't matter; don't forget to change the file filter from s19 to hex or bin or you won't see the firmware file). Leave the other options on the fourth page unchanged and click Next.

The flasher tool will flash the firmware, a progressbar will be shown and when finished, you can close the tool. You have now flashed the firmware successfully, congratulations. If you soldered the boot pads together, you'll need to remove the bridge now. Remove all the cables from the CC3D board.

# Connecting the boards together
Connect the CC3D to a computer with a USB cable. The Status LED of the board should light up and the computer should recognize the device as a gamepad / joystick and install the default generic drivers - these are fine, no need for any special drivers. Connect the receiver port harness to the CC3D board and connect your PPM receiver to CH1 on the board. The CC3D board can give power to the receiver through the harness as well, so the whole thing is powered only from the USB port, no need for external power supply. Your joystick is now fully functional.

# Compiling
To compile this project, you will need:
- GNU ARM compiler toolset: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
- Make tool (default in most Linux distros)
- (Optional) STM's CubeMX IDE, if you plan to change pin assignments or add functionality to other pins: http://www.st.com/en/development-tools/stm32cubemx.html

The makefile for this project should compile out of the box. If your GNU ARM compiler is not in path, you can use the `BINPATH` variable to point to the correct location:
```make BINPATH=/home/user/tools/gcc-arm-none-eabi-6-2017-q2-update/bin```
The build process will place the firmware files into the `build` folder

You can use the CubeMX IDE to modify the project settings, add new pin assignments etc., just open the CC3D-PPM-Joystick.ioc file. Note, however, that if you use the IDE to re-generate the project (i.e. regenerate the Makefile or create a project setup for other IDEs), you will need to revert its changes to the Middleware folder - there is one change to the USB HID class driver there that needs to be kept as-is, otherwise the board will appear as a mouse, rather than a joystick (The USH HID descriptor lives in those library files, unfortunately).

# Acknowledgements
This project is inspired by @alexeystn's stm32-ppm-usb-adapter ( https://github.com/alexeystn/stm32-ppm-usb-adapter ), which was used as the base for all the user code in the firmware. It has been altered for the CC3D hardware and the code was cleaned up for better understandability. Thanks, Alexey!

Big thanks to southquay3d.com for their "CC3D flashing tutorial", I wouldn't have found out how to flash firmware to the CC3D board without their Betaflight flashing tutorial: http://www.southquay3d.com/index.php?route=news/article&news_id=9
