# Preseason Planning
 - Modularity through layers
   1. Input layer
      - Directly pipe controller input
      - Simulated / Recorded (?) inputs
   2. Action layer
      - Receive input and act accordingly
      - Should be structured reactively, as in not dependent on inaction
      - Action layer will output a set of commands for corresponding physical elements of the robot
   3. Control layer
      - Receive commands from the Action layer
      - This layer is where physical components (ie. motors, servos, etc.) get acted upon
   - This system will increase control and flexibility of the codebase
 - Autonomous
   - With the exception of the drivetrain, a system that can record use inputs could be used to operate the robot autonomously
     - If not, a tree like system would be best, similar to that of last year
   - The drivetrain will use odometry and PIDs to move along Bezier curves during the Auto phase. 