/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@Autonomous(name="Autonomous", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor1 = null;
    DcMotor leftMotor2 = null;
    DcMotor rightMotor1 = null;
    DcMotor rightMotor2 = null;
    DcMotor catapultMotor = null;
    DcMotor intakeMotor = null;
    DcMotor liftMotor1 = null;
    DcMotor liftMotor2 = null;

    Servo autoMod = null;
    Servo buttonPress = null;
    Servo plateRelease = null;

    ColorSensor colorSensorL;
    ColorSensor colorSensorR;
    TouchSensor touchSensor;  // Hardware Device Object
    IrSeekerSensor irSeeker;    // Hardware Device Object


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor1 = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor2 = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");
        catapultMotor = hardwareMap.dcMotor.get("catapultMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        //servo hardware map
        autoMod = hardwareMap.servo.get("autoMod");
        buttonPress = hardwareMap.servo.get("buttonPress");
        plateRelease = hardwareMap.servo.get("plateRelease");

        //sensor hardware map
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        irSeeker = hardwareMap.irSeekerSensor.get("irSeeker");
        colorSensorL = hardwareMap.colorSensor.get("sensor_colorL");
        colorSensorR = hardwareMap.colorSensor.get("sensor_colorR");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        //liftMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //for joystick

        double irAngle;
        double irStrength;
        long startTime;
        long time;
        long autoProgress;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        startTime = System.currentTimeMillis();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            time = System.currentTimeMillis();
            autoProgress = time-startTime;

            telemetry.addData("Autonomous Progress", autoProgress);

            telemetry.addData("rightMotor1", rightMotor1.getPower());
            telemetry.addData("rightMotor2", rightMotor2.getPower());
            telemetry.addData("leftMotor1", leftMotor1.getPower());
            telemetry.addData("leftMotor2", leftMotor2.getPower());

            telemetry.addData("liftMotor1", liftMotor1.getPower());
            telemetry.addData("liftMotor2", liftMotor2.getPower());

            telemetry.addData("intakeMotor", intakeMotor.getPower());

            telemetry.addData("autoMod Position", autoMod.getPosition());
            telemetry.addData("buttonPress Position", buttonPress.getPosition());
            telemetry.addData("plateRelease Position", plateRelease.getPosition());

            float hsvValues[] = {0F, 0F, 0F};
            final float values[] = hsvValues;

            telemetry.addLine("colorSensorR");
            telemetry.addData("Clear", colorSensorR.alpha());
            telemetry.addData("Red  ", colorSensorR.red());
            telemetry.addData("Green", colorSensorR.green());
            telemetry.addData("Blue ", colorSensorR.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.addLine("ColorSensorL");
            telemetry.addData("Clear", colorSensorL.alpha());
            telemetry.addData("Red  ", colorSensorL.red());
            telemetry.addData("Green", colorSensorL.green());
            telemetry.addData("Blue ", colorSensorL.blue());
            telemetry.addData("Hue", hsvValues[0]);

            if (touchSensor.isPressed())
                telemetry.addData("touchSensor", "Pressed");
            else
                telemetry.addData("touchSensor", "Not Pressed");

            if (irSeeker.signalDetected()) {
                // Display angle and strength
                telemetry.addData("Angle", irSeeker.getAngle());
                telemetry.addData("Strength", irSeeker.getStrength());
            }
            else {
                // Display loss of signal
                telemetry.addData("Seeker", "Signal Lost");
            }

            telemetry.update();

            if (autoProgress < 1.5) {
                leftMotor1.setPower(1);
                leftMotor2.setPower(1);
                rightMotor1.setPower(1);
                rightMotor2.setPower(1);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
