# Car-Controller
A* based car controller that has been modified to reduce the amount of waypoints and improve track completition time.

The framework system called “Applying EANNs” used for
this project has been downloaded from GitHub site
belonging to Samuel Arzt, initial system had neural
networks algorithm applied to control vehicles, however,
the AI was removed, and own artificial intelligence has
been applied. Other parts of the system responsible for
creating cars, tracks and UI have been left as they were.
Original license for the use of software can be found either in
this repository or at:
https://github.com/ArztSamuel/Applying_EANNs/blob/master/LICENSE

Original project by Samuel Arzt can be found at:
https://github.com/ArztSamuel/Applying_EANNs/

The AI pathfinding algorithms in this project has been developed by Marcin Watroba.

<h2>Description of the Controller</h2>
Algorithm creates an A* grid with usual passable and impassable nodes, however, semi-passable nodes have been added around impassable ones. This ensures that car is not driving too close to the walls, unless there is no other way. The first part of the line of sight algorithm removes A* generated waypoints based on the line of sight. The second part of the line of sight algorithm relocates nodes that are left to more optimal locations, again based on line of sight.

<p>Once path is generated, a simple automated car follows it using vector calculations to determine speed and turning angles. On the picture, impassable nodes can be seen in red, and semi- passable ones in blue. Path can also be seen as black lines connected by nodes which are represented as perpendicular lines. Car is the green rectangle in the middle.</p>
<img src="https://github.com/marcin388/Car-Controller/blob/master/carController.jpg">

