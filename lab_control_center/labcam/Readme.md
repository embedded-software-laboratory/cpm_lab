# Labcam Requisites {#LabcamReadme}

- Pylon SDK installieren 
- gstreamer 

sudo apt-get install libgstreamer*1.0* gstreamer*1.0*

## Camera Config

Im Ordner camera_config liegt eine *.pfs Datei, die auf die Kamera geladen werden muss. Dies kann per Pylon Viewer gemacht werden. Zuerst mit Kamera verbinden, dann die pfs-Datei über Camera -> Load Features einmal laden. Um die Einstellung persistent zu machen, muss in den Einstellungen der Kamera unter Configuration Sets zunächst User Set 1 ausgewählt wird, dann "User Set Save" ausführt und unter Default Startup Set "User Set 1" auswählt. 