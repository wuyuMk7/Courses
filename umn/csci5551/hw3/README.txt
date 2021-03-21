Name: Yang He
X500: he000242

Package being used: 
rospy, tf, nav_msgs, geometry_msgs, sensor_msgs, std_srvs
sys, select, termios, tty, math, enum

Planning algorithm:
Mainly Bug2 algorithm.

(1) The agent draws an M-line between the current position and the goal position. 
(2) Then the agent turns to face the goal position and moves straight along the M-line. 
(3) If there is an obstacle in front of the agent, the agent will record current position 
    as the obstacle entry point and then move along the boundary of the obstacle.
    - If the agent moves onto the M-line again and current position is closer to the goal position 
      than the obstacle entry point, then the agent will turn to face the goal position and 
      move away from the obstacle.
    - If the agent moves onto the M-line again and current position is not closed to the goal position 
      than theobstacle entry point, the agent will keep moving along the obstacle boundary.
    - If the agent reaches the obstacle entry point again, which means the goal position is unreachable, 
      then the current run will be terminated.
(4) If the agent can leave from the obstacle, then the agent will go back to step2: to fix its 
    walking direction first, and then move staright to the goal until it reaches an obstacle.
      
