- This package serves as a robot manager. This code integrates the [warehouse manager](https://github.com/KrishnaBhatu/warehouse_manager) which performs the task of assigining goals with the local path planning and collision avoidance method [CADRL](https://github.com/mit-acl/cadrl_ros)

- This package also integrated a global planner method [currently A-star](https://github.com/lh9171338/Astar) 

- Launch the node with appropriate robot namespace eg. `<node pkg="robot_manager" type="robot_master" name="robot_master" output="screen" ns="robot_0"/>`

