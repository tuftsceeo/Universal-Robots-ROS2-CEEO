# UR Teach Pendant Tutorial
Each Universal Robot model is accompanied by a Teach Pendant; a touch screen display that interfaces with the robot. We will use the Teach Pendant to start up the robot, monitor the robot while in use, and put the robot into freedrive mode.

## Starting the Robot
1. Press the white power button to turn on the Teach Pendant.
2. Click on the red circle in the lower left hand corner of the screen.
<div style="display: flex; align-items: center;">
    <img src="examples/img/step1.png" alt="Image 1" style="width:300px;"/>
</div>
3. Click "ON" to give power to the robot.
<div style="display: flex; align-items: center;">
    <img src="examples/img/step2.png" alt="Image 1" style="width:300px;"/>
</div>
4. Click "START" to release the joint locks and enable the robot's motion.
<div style="display: flex; align-items: center;">
    <img src="examples/img/step3.png" alt="Image 1" style="width:300px;"/>
</div>
5. Click "Exit" to return back to the home screen.
<div style="display: flex; align-items: center;">
    <img src="examples/img/step4.png" alt="Image 1" style="width:300px;"/>
</div>
6. Click "Load Program."
<div style="display: flex; align-items: center;">
    <img src="examples/img/step5.png" alt="Image 1" style="width:300px;"/>
</div>
7. Select the "external control.urp" program and then click "Open."
<div style="display: flex; align-items: center;">
    <img src="examples/img/step6.png" alt="Image 1" style="width:300px;"/>
</div>
8. Click the big blue play button to run the program.
<div style="display: flex; align-items: center;">
    <img src="examples/img/step7.png" alt="Image 1" style="width:300px;"/>
</div>

## Monitoring the Robot
While the robot arm is in use, the Teach Pendant should be in an easily accessed location. If you need to stop the robot while it is in motion, you can either:
1. Press the big red E-Stop button on the Teach Pendant. This will stop power to the robot arm, and require you to start up the robot again.
2. Press the pause button at the center of the program page on the Teach Pendant or at the bottom right hand corner of every other page.

If an error shows up on the Teach Pendant, for example "Protective Stop," follow these steps to return to operating the robot.
1. Ensure that your control program has stopped running. Use Ctrl ^C to force quit a frozen process.
2. Click "Enable Robot" on the Teach Pendant
3. Click the blue play button to resume external control.

## Using Freedrive
Freedrive is a useful tool to manually reposition the robot arm or record a trajectory. To enter freedrive mode, follow these steps.
1. Click the blue pause button to pause external control.
2. Press and hold the black tactile button the the back front of the Teach Pendant.
3. Or, navigate to the "Move" page on the Teach Pendant and click and hold the "Freedrive" button.
<div style="display: flex; align-items: center;">
    <img src="examples/img/step9.png" alt="Image 1" style="width:300px;"/>
    <img src="examples/img/step10.png" alt="Image 1" style="width:300px;"/>
</div>
