# ur5_python_host
## Description:

simple general purpose ur5 controller


 ## Connection Setup:

-copy contents of kg_client to a memory stick and save program to ur5

-self.host: host pc ip address, this is the address of the socket connection (windows use ipconfig in cmd), the address in the kg_client.urp program must match this

-port:  socket port for the robot, use 30000 or >=30010 (29999 reserved for dashboard, 30001-30004 reserved for data exchange) set in kg_client.urp (different for every robot sharing a host)

-ee_port: e.g. 'COM20'

-db_host: socket address for robot, e.g. '192.168.1.10', set to enable dashboard, set in robot network settings:  
--in static address, set ip to 192.168.1.xx where the first 3 numbers match the self.host address and the last number differs, 
--set mask to 255.255.255.0
--apply


## Troubleshooting:

-have done everything above, and robot still not connecting - turn off wifi and/or disable other network connections
    

## Getting Started:
-main loop in \ur5_kg_robot\ur5_kg_robot.py, contains examples of using kg_robot

-create specialised robot fns using formatting from \ur5_kg_robot\specialised_kg_robot_example.py

-use waypoint.py for global robot poses, joints and tool centre points (tcp)
