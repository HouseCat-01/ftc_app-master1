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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="FirstOpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class FirstOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor1 = null;
    DcMotor leftMotor2 = null;
    DcMotor rightMotor1 = null;
    DcMotor rightMotor2 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor1  = hardwareMap.dcMotor.get("leftMotor1");
        leftMotor2  = hardwareMap.dcMotor.get("leftMotor2");
        rightMotor1 = hardwareMap.dcMotor.get("rightMotor1");
        rightMotor2 = hardwareMap.dcMotor.get("rightMotor2");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        //for joystick
        double left;
        double lefttotal;
        double right;
        double righttotal;
        double backdiag;
        double backdiagtotal;
        double frontdiag;
        double frontdiagtotal;
        double rightMotor1power;
        double rightMotor2power;
        double leftMotor1power;
        double leftMotor2power;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            rightMotor1power = rightMotor1.getPower();
            rightMotor2power = rightMotor2.getPower();
            leftMotor1power = leftMotor1.getPower();
            leftMotor2power = leftMotor2.getPower();
            telemetry.addData("rightMotor1", rightMotor1power);
            telemetry.addData("rightMotor2", rightMotor2power);
            telemetry.addData("leftMotor1", leftMotor1power);
            telemetry.addData("leftMotor2", leftMotor2power);

            telemetry.update();

            // drive system
            left = -gamepad1.left_stick_y - gamepad1.left_stick_x;
            lefttotal = left/2;
            right = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            righttotal = right/2;
            frontdiag = gamepad1.right_stick_x;
            frontdiagtotal = (frontdiag*1.2)/2;
            backdiag = gamepad1.right_stick_x;
            backdiagtotal = (backdiag)/2;

            if ((gamepad1.left_stick_x <= 0.05) && (gamepad1.left_stick_y <= 0.05) && (gamepad1.left_stick_x >= -0.05) && (gamepad1.left_stick_y >= -0.05) && (gamepad1.right_stick_x <= 0.03) && (gamepad1.right_stick_x >= -0.03)) {
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