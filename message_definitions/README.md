# MAVLink extension messages
the agrodrone.xml file is used to extend the ardupilotmega.xml mavlink file

just <include>agrodrone.xml</include> into the ardupilotmega.xml

Then use the generator included in the mavlink package to generate the correct headers.

### create pymavlink package

```
$ cd mavlink/pymavlink
$ python setup.py install # maybe add sudo
```
