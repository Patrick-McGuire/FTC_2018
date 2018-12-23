
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="CL_Teleop", group="Linear Opmode")
public class CL_Teleop extends LinearOpMode {

    // Declare OpMode members.
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    Orientation angles;

    public double angleOfset = 0;

    @Override
    public void runOpMode() {

        double Kp = .1;
        double Ki = .000106;
        double Kd = 8.6;
        double targetAngle = 0;

        double steering = 0;

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
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double angle = getAngle();
            steering = -gamepad1.left_stick_x;

            if (steering != 0) {
                runtime.reset();
            }
            if (runtime.milliseconds() < 300) {
                targetAngle = angle;
            } else if (runtime.milliseconds() > 300) {
                steering = driveTrain_PID.runPID(targetAngle, angle);
            }

            double drivePower = gamepad1.right_trigger - gamepad1.left_trigger;
            driveTrain.driveSteering(drivePower, steering);

            if (gamepad1.dpad_left) {
                zeroGyro();
                targetAngle = 90;
            }
            if (gamepad1.dpad_right) {
                zeroGyro();
                targetAngle = -90;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("stuff", " input (%.2f), steering power (%.2f), Kp (%.2f, Goal (%.2f), Kd (%.2f), Ki (%.2f), error (%.2f)", angle, steering, Kp, targetAngle, Kd, Ki * 100000, targetAngle-angle);
            telemetry.update();
        }
    }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle;

        deltaAngle -= angleOfset;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        
        return(deltaAngle);
    }

    public void zeroGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleOfset = angles.firstAngle;
    }
}
