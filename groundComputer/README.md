The ground control script runs on your team's laptop, listening to a second LoRa module plugged into a USB port. use python

The script uses pyserial to read the live radio text stream coming through the USB port, and vpython (a 3D rendering library) to render a live, interactive 3D environment.


Key Python Functions & Logic Explained

1. Canvas & Object Setup (vpython):
    -Creates a dark background viewport and draws an orange 3D cylinder (cansat) representing the physical fiberglass body tube.
    -Defines scene.up = vector(0,0,1) to establish standard aerospace coordinate space ($Z$-axis pointing straight up into the sky).

2. Serial USB Listening Loop (pyserial):
    -[ser.readline()]: Reads incoming raw bytes from the USB port and converts them into an ASCII string.
    -[packet.startswith("$CANSAT") and packet.endswith("*")]: Filters out garbled packets caused by background radio static or interference, ensuring only clean data gets processed.

3. Data Parsing & Trigonometric Math:
    -[.replace()] & [.split(",")]: Strips out header tags and splits the comma-separated string into an array of individual numbers (packet_id, ax, ay, az, etc.).
    -Vector Calculations (math.atan2): Uses basic trigonometry on the raw $X, Y, Z$ acceleration forces to compute exact spatial tilt angles:
       $$\text{Pitch} = \arctan2\left(A_x, \sqrt{A_y^2 + A_z^2}\right)$$
       
       $$\text{Roll} = \arctan2\left(A_y, \sqrt{A_x^2 + A_z^2}\right)$$

4. 3D Frame Rendering:
    -[new_axis_x], [new_axis_y], [new_axis_z]: Combines the calculated Pitch and Roll angles into a 3D unit vector representing the rocket's current spatial orientation.
    -[cansat.axis = vector(...)]: Updates the 3D cylinder model's spatial orientation in the software window.
    -[rate(60)]: Locks the program loop to $60\text{ frames per second}$, creating a smooth visual rendering of the rocket tumbling, ascending, and descending in real time on your laptop screen.