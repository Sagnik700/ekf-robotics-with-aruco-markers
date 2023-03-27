# Practical on computer vision and robotics

A. This is the main epic for the whole practical project
B. We would create issues under this project and then merge the code with respective pull requests

1 Getting started
This chapter concerns hardware and software matters to get started. It covers information from the
ev3 to the raspberry pi and how to interface with your PC.
1.1 Connecting the hardware
The motors plug into the A, B, C, D ports of the ev3. The sensors plug into the 1, 2, 3, 4 ports
of the ev3.
The lineleader sensor and the IMU (accelerometer, compass and gyro) sensor are currently not
supported. If someone wants to use these they can implement a sensor class for this and then please
share it with the group and make a pull-request on github :).
The ev3 has a battery pack that connects directly into the ev3. This is charged via the 10v power brick
and connects via a barrel plug. The raspberry pi is powered over usb-c with the usb-c power brick.
Use the usb-c power bank to power the raspberry pi when using the robot wirelessly. Connect the
ev3 to the raspberry pi using the usb-A to usb-B cable. The usb-A goes into the raspberry pi and the
usb-B goes into the ev3. The ev3 acts as the slave and the raspberry pi acts as the master. The micro
SD card is loaded with raspberry OS with some extra software from us and it should be plugged into
the raspberry pi. The ev3 runs its built in firmware and doesn’t need a micro SD card.
For debugging we recommend running the ev3 via the charging cable. With the ev3 battery the
motors will only perform at their full performance for 10 to 20 minutes. The raspberry pi can run for
6 hours with a full charge of the battery bank.
The webcam connects to the raspberry pi.
1.2 Raspberry Pi
Specification: Raspberry Pi 4, 4GB running Raspberry Pi OS, 64bit with some additional software
installed to get started quickly.
Useful utilites:
• uprecords to look at uptime.
• /home/pi/scripts/undervolt_protection.sh shutdown raspberry pi when the voltage drops too much. Runs every 15 minutes via a crontab -e
• X11Forwarding yes set in /etc/ssh/sshd_config
Warning! Don’t press button on battery pack. This will stop the power to the raspberry.
1.2.1 Wifi Connect
The raspberry spawns a hotspot if it cannot connect to wifi. It uses wifi-connect and wifi-connect on
startup using systemd (links for the curious).
1
The SSID is: pi-<mac address without colons>
The password is: device-<last four chars of mac address>
Once you have logged in to the wifi hotspot, you will be presented with a captive portal where you
can connect to your own wifi network. Eduroam is not supported.
Login to your router and find out which ip it has given to the raspberry. You can then get your router
reserve this ip for the raspberry.
Figure 1.1: DHCP IP address reservation on a D-Link router.
Note down this ip address. It will be something like 192.168.0.102 as in our case.
1.3 SSH client and VS Code
We advise to program on the raspberry pi via VS Code.
1.3.1 Linux and Mac
Create a file ~/.ssh/config and append the following to it (change the ip address as appropriate):
Host raspberry
HostName 192.168.0.102
User pi
ForwardX11Trusted yes
ForwardX11 yes
Now in your terminal you should be able to SSH into the raspberry by doing
$ ssh raspberry
The default username is pi and the default password is raspberry.
Install VSCode. On Linux I would suggest installing via the apt-get package. Once in VS Code,
install the remote SSH extension and access the raspberry via the status bar item in the far left corner.
When accessing the raspberry pi via VS Code, you can you can install the python extension.
2
To enable passwordless ssh access to your raspberry pi, you can use a public-private key pair. On
your computer run the following two commands:
$ ssh-keygen -t rsa -b 4096
$ ssh-copy-id -i ~/.ssh/id_rsa.pub raspberry
1.3.2 Windows
For Windows the process is very similar, but you might have to install openSSH client first. See this
guide for more information. Note: you don’t need the Windows OpenSSH server.
1.4 Tensorflow
Tensorflow is compiled if you really want it.
2 Programming on Windows
By following these steps you should be able to program the ev3 robot under Windows 10.
1. Install Anaconda here.
2. Open Anaconda Prompt (.anaconda3) from the Windows start menu.
3. Create a new environment and install the ev3-python3 package by running:
$ conda create -n ev3 python=3.8
$ conda activate ev3
$ pip3 install ev3_dc
4. Copy paste the demo code below into a file called ev3_demo.py.
5. Download and install libusb-win32.
6. Run C:\Program Files\LibUSB-Win32\bin\install-filter-win.exe (this will actually run after installing).
7. Plug in and turn on the ev3 to your computer. Click Install a device filter. Look which
devices are there. Unplug the ev3, and click back and then again on Install a device filter.
Look which usb device corresponds to the ev3. Select this device and click Install.
8. In the conda environment run
$ python ev3_demo.py
It should be running now. You should install VS Code and the python extension for VS code if you
haven’t done so already.
To install opencv run:
$ pip3 install opencv-python
3 Programming on Mac OS or Linux (e.g. Ubuntu 20.04)
Follow these steps to run the ev3 under Mac OS or Linux:
1. Install Anaconda from here.
2. Copy paste the demo code below into a file called ev3_demo.py.
3. Create a new environment and install the ev3-python3 package by running:
$ conda create -n ev3 python=3.8
$ conda activate ev3
$ pip3 install ev3_dc
$ pip3 install hidapi # this is only required for Mac!
$ python ev3_demo.py
3
It should be running now. You should install VS Code and the python extension for VS code if you
haven’t done so already.
For Linux (NOT Mac OS) you may need to add udev rules. Without the udev rules, sudo is needed so
that python can access the HIDraw.
To gain access to the brick as a normal user, put these udev rules into place by putting them into a
rules file in /etc/udev/rules.d/ and refresh udev with sudo udevadm control --reload.
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0694", ATTRS{idProduct}=="0005", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", ATTRS{idProduct}=="0005", TAG+="uaccess"
To install opencv run:
$ pip3 install opencv-python
4 Programming on the Raspberry Pi
Open VS Code and connect to the raspberry pi. Go to Terminal → New Terminal, to open a terminal.
The raspberry pi has python 3.7.3 installed. You can install extra packages using
$ pip3 install scipy (or whatever package you like)
4.1 The ev3-python3 Package
We use the ev3-python3 package to control the ev3 from the raspberry pi. The documentation can be
found here.
The ev3-python3 package is already installed and has been done so via pip3 install ev3_dc.
To control two motors plugged into port A and port D you can do:
import ev3_dc as ev3
with ev3.EV3(protocol=ev3.USB) as my_ev3:
motor_a = ev3.Motor(ev3.PORT_A, ev3_obj=my_ev3)
motor_d = ev3.Motor(ev3.PORT_D, ev3_obj=my_ev3)
motor_a.start_move_by(360*2, speed=80, brake=True)
motor_d.start_move_by(360*2, speed=80, brake=True)
To get the colour from the colour sensor you can do:
import ev3_dc as ev3
with ev3.EV3(protocol=ev3.USB) as my_ev3:
print(my_ev3.sensors)
my_color = ev3.Color(ev3.PORT_4, ev3_obj=my_ev3)
print('The color is', my_color.color)
print('The reflected intensity is ', my_color.reflected, '%')
print("The ambient colour is", my_color.ambient)
To get the touch sensor value you can do:
import ev3_dc as ev3
with ev3.EV3(protocol=ev3.USB) as my_ev3:
my_touch = ev3.Touch(ev3.PORT_1, ev3_obj=my_ev3)
print('touched' if my_touch.touched else 'not touched')
4
4.2 OpenCV and the Webcam
The package opencv-contrib-python, version 4.5.3 is installed already. To get images from the camera,
run:
import cv2
def show_webcam():
cam = cv2.VideoCapture(0)
while True:
ret_val, img = cam.read()
cv2.imshow('my webcam', img)
if cv2.waitKey(1) == 27:
break # esc to quit
cv2.destroyAllWindows()
if __name__ == '__main__':
show_webcam()
You should see the live camera feed on your computer. You are receiving this feed via X11.
4.2.1 X11 Forwarding
You may need to install the Remote X11 extension in VS Code.
On Mac you need to install xquartz, keep it running in the background whenever you need X11, and
in the preferences enable Security → Authenticate Connections, and Allow connections from network
clients.
On Linux X11 forwarding on the client should work out-of-the-box.
For X11 on Windows you either need PuTTY client or Xming. See this guide for more details.
5