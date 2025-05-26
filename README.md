# rob499_rover_status_ui  


Instructions:

Required imports:  

Once packages have been built and sourced, the Status UI tool info nodes and demo nodes can be run with:  

>ros2 launch rob499_rover_status_ui rover_ui.py  


In another terminal to run the actual UI run:  

>ros2 run rob499_rover_status_ui integrator --ros-args --disable-stdout-logs  





How to select node for detailed view:  
>  


Note: when viewing node details, if the subscribers/publishers/services change after the node selection service call is ran, the details will not update until the service call for the node is ran again  
