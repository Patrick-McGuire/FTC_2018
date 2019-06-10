
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="DT_700_Control", group="Linear Opmode")
public class DT_700_Control extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo servo1 = null;



    @Override
    public void runOpMode() {


        servo1 = hardwareMap.get(CRServo.class, "servo1");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double motorPower = gamepad1.left_stick_y;

            servo1.setPower(motorPower);

            //telemetry.addData("stuff", " armPower (%.2f), armEncoder (%.2f), Kp (%.2f, Goal (%.2f), Kd (%.2f), Ki (%.2f), error (%.2f), pitch (%.2f)", armPower, test, Kp, targetAngle, Kd, Ki * 100000, targetAngle-angle, getPitch());
            //telemetry.update();
        }
    }


}
