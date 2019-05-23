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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.SynchronousQueue;

@Autonomous(name="thing", group="Linear Opmode")
public class Thread_test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    private DcMotor arm = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    Orientation angles;



    //Thread stuff
    public DataPass DataPassz = new DataPass();
    private Thread1 Thread1z = new Thread1(DataPassz);

    public double angleOfset = 0;

    @Override
    public void runOpMode() {

        double KpArm = .005;
        double KiArm = 0;
        double KdArm = .4;

        double KpDist = .005;
        double KiDist = 0;
        double KdDist = .4;

        double KpStr = .01;
        double KiStr = .0;
        double KdStr = 0;

        //#0 arm goal
        //#1 distance
        //#2 angle
        int[] goals;
        int[] Inputs = new int[3];

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        PID arm_PID = new PID(KpArm, KiArm, KdArm);
        PID distance_PID = new PID(KpDist, KiDist, KdDist);
        PID steering_PID = new PID(KpStr, KiStr, KdStr);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Stuff for hardware map
        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Set default motor direction
        arm.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        leftDrive.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        tankDrive driveTrain = new tankDrive(rightDrive, leftDrive);

        zeroGyro();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        Thread1z.start();

        arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        leftDrive.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        rightDrive.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        while (opModeIsActive()) {

            double angle = getAngle();
            Inputs[0] = arm.getCurrentPosition();
            Inputs[1] = leftDrive.getCurrentPosition();
            Inputs[2] = (int) angle;

            goals = DataPassz.getGoals();
            DataPassz.setInputs(Inputs);

            double angleGoal = goals[2];

            if (angleGoal > 150 && angle < 0 && angle > -181){
                angle = angle + 360;
                angleGoal = angleGoal + 360;
            }
            if (angleGoal < -150 && angle > 0 && angle < 181){
                angle = angle - 360;
                angleGoal = angleGoal - 360;
            }

            double armPower = arm_PID.runPID(goals[0], arm.getCurrentPosition());
            arm.setPower(armPower);

            double drivePower = distance_PID.runPID(goals[1], leftDrive.getCurrentPosition());
            double steering = steering_PID.runPID(angleGoal, angle);

            driveTrain.driveSteering(drivePower, steering);

            double error = angleGoal - angle;
            double pwr =  rightDrive.getPower();

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double rawange = angles.firstAngle;


            telemetry.addData("stuff", " error (%.2f), pwr (%.2f), angle (%.2f), angle (%.2f)", error, pwr,angle,rawange);
            telemetry.update();
        }
        }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle;

        deltaAngle -= angleOfset;

        //if(deltaAngle < 0) {
         //   deltaAngle += 360;
        //}

        return(deltaAngle);
    }

    public void zeroGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleOfset = angles.firstAngle;
    }
    }