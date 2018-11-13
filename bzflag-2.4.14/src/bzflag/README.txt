Assignment 3 README

Copy the following files into bzflag-2.4.14\src\bzflag
 RobotPlayer.cxx
 RobotPlayer.h
 Pathfinder.cpp
 Pathfinder.h

These mods use the YAGSBPL library.


Notes:
	In RobotPlayer.cxx I modified the doUpdateMotion function to call a pathfinding function
	in Pathfinder.cpp. There is logic to determine first, what target the tank is looking for
	and then second, if the target has moved at all.

	This is done because the cost to pathfind is relatively significant and causes slowdown if
	done frequently. This way it is only called if necessary in times where the tank is looking
	for a new target or if the target that they were currently seeking has moved.

	This informaiton is held in the variable "float currentTarget" that is initialized to {0,0,0}.
	On death however, this does not reset to the initial value so there is also additional logic
	to determine if it is already seeking down a path.

	As far as issues go, due to the way the coordinates are translated from the world to the graph
	I have noticed occasional instances where the tank could not reach it's target and would circle
	around to try again continuously. Currently the coordinates of the game world are divided by
	the tank size to have a smaller more manageable graph to pathfind through however this float value
	that I'm using to determine the positions is being rounded to the nearest integer. Initially I was
	truncating the values after the decimal but that caused the problem to become even more prominent.
	The current implentation of rounding is much more reliable however still not foolproof.