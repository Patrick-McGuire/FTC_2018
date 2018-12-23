/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="1Controller", group="Linear Opmode")
public class One_Controller_Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private DcMotor climber = null;
    private CRServo servo1 = null;
    private CRServo servo2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        climber = hardwareMap.get(DcMotor.class, "climber_motor");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        // Set up default motor direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        climber = hardwareMap.get(DcMotor.class, "climber_motor");

        tankDrive driveTrain = new tankDrive(rightDrive, leftDrive);

        arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

            // Setup a variable for each drive wheel to save power level for telemetry
            double armPower;
            double climbPower;
            int intakePower;

            int encoderValue = arm.getCurrentPosition();
            boolean other = true;

            // Math for controlling the robot
            double drivePower = gamepad1.right_trigger - gamepad1.left_trigger;
            double steering  =  gamepad1.left_stick_x;
            double armMove = gamepad1.right_stick_y;
            armPower   = Range.clip(armMove, -1.0, 1.0) ;

            // Intake control
            if (gamepad1.a) {
                intakePower = 1;
            }
            else
            if (gamepad1.b) {
                intakePower = -1;
            }
            else
            {
                intakePower = 0;
            }

            // Climber control
            if (gamepad1.dpad_up) {
                climbPower = 1;
            }
            else
            if (gamepad1.dpad_down) {
                climbPower = -1;
            }
            else
            {
                climbPower = 0;
            }

            telemetry.addData("encoderValue", arm.getCurrentPosition());

            if (-.1 < gamepad1.right_stick_y && gamepad1.right_stick_y < .1) {
                telemetry.addData("yes",0);
                other = false;
                arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
                if (encoderValue<-25) {
                    arm.setPower(.2);
                    telemetry.addData("yes2",0);
                    other = true;
                }
                else if (encoderValue>25) {
                    arm.setPower(-.2);
                    telemetry.addData("yes3",0);
                    other = true;
                }
                else {
                    arm.setPower(0);
                    telemetry.addData("yes4",0);
                    //other = true;
                }
            }
            else {
                //arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
                other = true;
            }

            // Send calculated power to wheels
            driveTrain.driveSteering(drivePower, steering*-1);

            climber.setPower(climbPower);
            servo1.setPower(intakePower);
            servo2.setPower(intakePower*-1);
            if (gamepad1.right_stick_y > .1) {
                telemetry.addData("no",0);
                arm.setPower(armPower*.7);
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            }
            if (gamepad1.right_stick_y < -.1) {
                telemetry.addData("no", 0);
                arm.setPower(armPower * .7);
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            }

            //Reset arm eancoder
            if (gamepad1.y) {
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            }
            //Reset arm eancoder
            if (gamepad1.dpad_down) {
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            }//Reset arm eancoder
            if (gamepad1.dpad_up) {
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
            }

                // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f), arm (%.2f), climber (%.2f)", leftPower, rightPower, armPower, climbPower);
            telemetry.update();
        }
    }
}