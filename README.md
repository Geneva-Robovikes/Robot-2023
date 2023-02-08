# Robot-2023
Geneva Robovikes code for the 2023 FRC season. I am sorry...

## **How to Run the Robot**
1) Connect to the radio over wifi*, which should end in 3067.
2) Open driver station and verify the connection box is green.
3) Using VS Code, click on the "W" in the top right then select "Deploy Robot Code".
4) Once the code is deployed, go back to driver station and wait for the text, "No Robot Code", to switch to "Disabled".
5) Check to make sure you are on the correct robot mode.
6) Press "Enable" to start the Robot!

*If things get a bit spicy, press space for an emergency stop. To restart the robot after, press the restart button on the roboRIO.*

*You do not need to wait to connect, as soon as you hit the button you are good.*

## **How to Make an Auto Path**
1) Open PathPlanner.
2) Make sure you have the correct project repository selected.
3) To make a new path select the new path option in the menu.
4) Chnge the path by dragging the dots around.
5) Click on a dot to see finer details.
6) Use the reversal option when the robot must enter in one direction and return the same way.
7) Use the stop option when you want the robot to stop and start a command.
8) The buttons at the bottom are used to edit the path, simulate the path, set markers, measue distances, and view the graph (in that order).
9) To have other commands run durring the path, go to the marker menu.
10) Click on one of the purple markers to select it.
11) In the menu that pops up, type the name of the event you want to execute in the Add new event box.
12) You can add as many events here as you like, keep note of the Execution Behavior when adding several events.
13) Use the Wait Behavior dropdown to customize how the robot's wait works.
14) Once the path is done, add any new commands used to the event hashmap and add the path to the dropdown on shuffleboard.

*Paths will automatically be added to the robot files. No need to save!*

## **Useful Links**
1) https://docs.wpilib.org/en/stable/ - Use this website to reasearch topics. It has great tutorials on trjectories, pathfinding, vision, and more if you look deep enough.
2) https://first.wpi.edu/wpilib/allwpilib/docs/release/java/index.html - Use this website when reasearching specific classes or methods, it gives code-sided descriptions.
3) https://www.chiefdelphi.com/ - Stack Overflow like website that focuses on FRC problems. Post a problem if in need!
4) http://discord.gg/frc - Unofficial FRC discord server. Lots of people willing to help with issues.
