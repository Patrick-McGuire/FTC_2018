
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "PID_Test", group = "Linear Opmode")
public class PID_Test extends LinearOpMode {

    BNO055IMU imu;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    Orientation angles;

    double Kp = .1;
    double Ki = .000006;
    double Kd = 9.6;
    double targetAngle = 0;

    @Override
    public void runOpMode() {
        PID driveTrain_PID = new PID(Kp, Ki, Kd);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //HardwareDevice.Manufacturer;
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        //Ser default direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        tankDrive driveTrain = new tankDrive(rightDrive, leftDrive);

        waitForStart();

        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //1120 tics per rev for 40:1
            targetAngle += (gamepad1.left_stick_x * 2);
            double angle = angles.firstAngle;

            double steering = driveTrain_PID.runPID(targetAngle, angle);

            driveTrain.driveSteering(0, steering);

            if (gamepad1.a) {
                Ki += -.000001;
            }
            if (gamepad1.b) {
                Ki += .000001;
            }
            if (gamepad1.dpad_up) {
                Kp += -.01;
            }
            if (gamepad1.dpad_down) {
                Kp += .01;
            }
            if (gamepad1.left_bumper) {
                Kd += -.1;
            }
            if (gamepad1.x) {
                Kd += .1;
            }
            telemetry.addData("stuff", " input (%.2f), motor output (%.2f), Kp (%.2f, Goal (%.2f), Kd (%.2f), Ki (%.2f)", angle, steering, Kp, targetAngle, Kd, Ki * 100000);
            telemetry.update();
        }
    }
}
