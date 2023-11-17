# RobWorkStudio Plugin

The SamplePlugin is a RobWorkStudio Plugin, meaning that this is not an executable, but a plugin that must be loaded by RobWorkStudio.

## Manually Load Plugin

To load a plugin manually select Plugins->"load plugin" in RobWorkStudio and then select the generated plugin.
For the provided SamplePlugin the file name is libRoViPlugin.so, here ".so" means that it is a library file and therefore not directly executable.

## Automatically Load Plugin

RobWorkStudio can automatically load your plugin when it starts, by providing it with an .RobWorkStudio.ini file in your home directory meaning "~/.RobWorkStudio.ini" or "/home/rovi2022/.RobWorkStudio.ini".

The file should contain the following to load:

``` ini
[Plugins]

RoViPlugin\DockArea=1
RoViPlugin\Filename=libRoViPlugin.so
RoViPlugin\Path=/path/to/the/pluginFolder
RoViPlugin\Visible=true
```

You can replace "RoViPlugin" with whatever you name your plugin.

- DockArea: is where the the Plugin should be placed in RobWorkStudio when it opens initially
- Filename: is the name of the Plugin file. OBS. no path only name
- Path:     is where the plugin is placed 
- Visible:  this is whether the plugin should be visible when RobWorkStudio is started or if you should click on the plugin icon to open it.

## Python plugin

The Python plugin needs to be loaded like any other c++ plugin by opening RobWorkStudio and Plugins->"load plugin". or by using the automatic method described in the previous section.

**OBS OBS OBS** the python plugins can only be loaded, by compiling RobWork with QT5, by default it is compiled with QT6, where some breaking changes was introduced for this feature. 

## Python StandAlone RobWorkStudio Runner

This is an alternative to running python inside RobWorkStudio. As it is running RobWorkStudio inside Python. You wont be able to create a gui, but you'll have all the other functions of RobWork.