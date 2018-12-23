
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="PID_Test", group="Linear Opmode")
public class PID_Test extends LinearOpMode {

    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    Orientation angles;

    double Kp = .15;
    double Ki = .000001;
    double Kd = 10;

    double p;
    double i;
    double d;

    double error;
    double goal = 90;

    double deltaTime;
    double lastTime = 0;
    double sum = 0;

    double outputa;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //HardwareDevice.Manufacturer;
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        //Ser default direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            //1120 tics per rev for 40:1
            //goal += (gamepad1.left_stick_y*10);
            double input = angles.firstAngle;
            double oldError = error;
            error = goal - input;

            deltaTime = runtime.milliseconds() - lastTime;
            lastTime = runtime.milliseconds();

            if (gamepad1.y) {
            //PID Math
            //P math
            p= Kp*error;

            //I math
            sum += error * deltaTime;
            i += Ki * sum;

            //D math
            d = Kd * ((error - oldError)/deltaTime);

            //PID calculations
            outputa = (p+i+d);


                leftDrive.setPower(-Range.clip((outputa), -1, 1));
                rightDrive.setPower(Range.clip((outputa), -1, 1));
            }
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sum = 0;
            }

            if (gamepad1.a) {
                Ki += -.000001;
            }
            if (gamepad1.b) {
                Ki += .000001;
            }
            telemetry.addData("stuff", "output (%.2f), input (%.2f), motor output (%.2f), Kp (%.2f, Goal (%.2f), Kd (%.2f), Ki (%.2f), i (%.2f)", outputa, input, outputa, Kp, goal, Kd, Ki,i);
            telemetry.update();
        }
    }
}