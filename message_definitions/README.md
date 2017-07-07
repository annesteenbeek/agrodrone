# MAVLink extension messages
the agrodrone.xml file is used to extend the ardupilotmega.xml mavlink file

just <include>agrodrone.xml</include> into the ardupilotmega.xml

Then use the generator included in the mavlink package to generate the correct headers.

### create pymavlink package
#### Note: bug
There is a bug where including a message induces a compilation error when validated, to disable validation during install, edit the mavlink/pymavlink/generator/mavgen.py and change DEFAULT_VALIDATE=True to False.

```
$ cd mavlink/pymavlink
$ python setup.py install # maybe add sudo
```
### Compiling pymavlink in windows
To compile pymavlink in windows, clone their github repo, then create a seperate message_def folder with the agrodrone specific message set included.
To compile, add the MDEV env variable
```
set MDEF=C:\Users\***\message_definitions\
C:\Python27\Scripts\pip install . -v -U # while in the pymavlink repo folder
```

### Include in qgroundcontrol
Make sure to use the recent version of the normal mavlink package, not the ROS version
Include the agrodrone.xml into the v1.0 message set (no symbolic link)
Get the mavlink submodule for pymavlink
build it and send the output to the v2.0 mavlink set in the qgroundcontrol package, rebuild

