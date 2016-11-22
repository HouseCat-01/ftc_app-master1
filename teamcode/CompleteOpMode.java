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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class CompleteOpMode extends LinearOpMode {

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

        IrSeekerSensor irSeeker;

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
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //for joystick
        double left;
        double lefttotal;
        double right;
        double righttotal;
        double backdiag;
        double backdiagtotal;
        double frontdiag;
        double frontdiagtotal;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

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

            telemetry.update();
            // drive system
            left = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            left = Range.clip(left, -1, 1);
            right = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            right = Range.clip(right, -1, 1);

            lefttotal = left;
            righttotal = right;

            frontdiag = gamepad1.right_stick_x;
            frontdiag = Range.clip(frontdiag, -1, 1);
            backdiag = gamepad1.right_stick_x;
            backdiag = Range.clip(backdiag, -1, 1);

            frontdiagtotal = (frontdiag);
            backdiagtotal = (backdiag);


            if ((gamepad1.left_stick_x <= 0.05) && (gamepad1.left_stick_y <= 0.05) && (gamepad1.left_stick_x >= -0.05) && (gamepad1.left_stick_y >= -0.05) && (gamepad1.right_stick_x <= 0.03) && (gamepad1.right_stick_x >= -0.03)) {
                //dead zones
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }
            else {
                //set power of motors
                leftMotor1.setPower(lefttotal - frontdiagtotal);
                leftMotor2.setPower(lefttotal + backdiagtotal);
                rightMotor1.setPower(righttotal + frontdiagtotal);
                rightMotor2.setPower((righttotal - backdiagtotal));
            }

            if (gamepad1.dpad_up) {
                autoMod.setPosition(0);
            }

            if (gamepad1.dpad_down) {
                autoMod.setPosition(1);
            }

            if (gamepad1.dpad_left) {
                buttonPress.setPosition(0.95);
            }
            else if (gamepad1.dpad_right) {
                buttonPress.setPosition(0.3);
            }

            if (gamepad1.right_trigger > 0.65) {
                plateRelease.setPosition(1);
                //wait(1000);
                //plateRelease.setPosition(0.5);
            }

            if (!touchSensor.isPressed()) {
                if (gamepad1.x) {
                    catapultMotor.setPower(-1);
                }
                else {
                    catapultMotor.setPower(0);
                }
            }
             else if (touchSensor.isPressed()) {
                if (gamepad1.right_bumper) {
                    catapultMotor.setPower(-1);
                    }
                else {
                    catapultMotor.setPower(0);
                    }

                if (gamepad1.b) {
                    intakeMotor.setPower(1);
                }
                else {
                    intakeMotor.setPower(0);
                }
             }
            else
                catapultMotor.setPower(0);

/*
            if (gamepad1.x) {
                catapultMotor.setPower(.7);
            }
*/

            if (gamepad1.y) {
                liftMotor1.setPower(-1);
                liftMotor2.setPower(1);
            }
            else if (gamepad1.a) {
                liftMotor1.setPower(1);
                liftMotor2.setPower(-1);
            }
            else {
                liftMotor1.setPower(0);
                liftMotor2.setPower(0);
            }
/*
            //backup code
            left = gamepad1.left_stick_y - gamepad1.left_stick_x;
            right = gamepad1.left_stick_y + gamepad1.left_stick_x;
            leftMotor1.setPower(left);
            leftMotor2.setPower(left);
            rightMotor1.setPower(right);
            rightMotor2.setPower(right);
*/

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
