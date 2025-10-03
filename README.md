This project presents a model predictive Controller (MPC) for a low-cost, line-following robot on Arduino Uno using a tree-based MPC and a Kalman filter for sensor fusion.
The Arduino Uno R3 was used and the mobile Robot used in this project is the Arduino Shield v1.2.

The program is optimized for tight RAM/flash budgets and real-time control and has the following features.

Tree-based MPC (discrete motion options) with the following cost-function set-up:

distance-to-path(m)· 300 

progress since last step(m) · 75

motion-option change (true or false) · 1
  
Kalman filter fusing Infrared sensory data with the motion model.
Adaptable prediction horizon (n), though increasing the prediction horizon may require freeing up some more memory.
A predefined path is already saved to the program and can be found in the Position on Line file of the program.
The program works on the processor ATmega328P @ 16 MHz, ~32 KB flash, ~2 KB SRAM.

To use the program it is recommended to use the Arduino IDE program. Simply download it, and within the IDE program open the file main_file.ino from the program.
With that all other components of the program open themselves automatically. Do not forget to install all the required libraries. 
After that connect your Arduino to your computer, select the correct board and port and upload the code and you are good to go.

Feel free to adapt and improve the code... enjoy.
