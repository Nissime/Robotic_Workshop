#  To install do the following:

#Install the required packages including QT5.

sudo apt-get update
sudo apt-get install qt5-qmake qt5-default \
  qtscript5-dev libqt5webkit5-dev libqt5serialport5-dev \
  libqt5svg5-dev qtdeclarative5-qtquick2-plugin
sudo apt-get install git libsdl1.2-dev  libsndfile-dev \
  flite1-dev libssl-dev libudev-dev libsdl2-dev
#Clone the git repo: (We'll clean it up shortly, don't worry)
git clone https://github.com/diydrones/apm_planner
#Make it and build it using QT.

cd apm_planner
qmake qgroundcontrol.pro
make
#If a dependency is needed while making, and you install it, you must perform qmake again then make. I had to install more QT dependencies and to edit a few QT files for QTWidgets.

#Run it.

./release/apmplanner2
#If the application did run (the GUI opened), this is great news. If it didn't, make sure make worked properly and there are no errors.

#Exit the application. Add your user to dialout group and remove the modem manager.

sudo adduser $USER dialout
sudo apt-get remove modemmanager
#Copy the whole folder to opt to clean things up.

sudo mv ../apm_planner /opt/
#Create an applications menu shortcut:

gedit ~/.local/share/applications/apm_planner.desktop
#And copy the following contents in it:
'''
[Desktop Entry]
Comment=
Terminal=false
Name=APM Planner
Exec=/opt/apm_planner/release/launch
Type=Application
Icon=/opt/apm_planner/release/files/APMIcons/ap_rc.png
'''
#Log out and in back again for the user group permissions to take effect.

#Run APM Planner from the Applications Menu.
