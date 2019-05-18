
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="CL_Teleop_arm", group="Linear Opmode")
public class CL_Teleop_Arm extends LinearOpMode {

    // Declare OpMode members.
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    Orientation angles;
    private DcMotor climber = null;
    private CRServo servo1 = null;
    private CRServo servo2 = null;

    public double angleOfset = 0;

    @Override
    public void runOpMode() {

        double Kp = .02; //.03;
        double Ki = 0; //.000106;
        double Kd = 0; //8;
        double targetAngle = 0;

        double steering = 0;

        double KpArm = .005;
        double KiArm = 0;
        double KdArm = .4;
        double armGoal = 0;
        PID driveTrain_PID = new PID(Kp, Ki, Kd);
        PID arm_PID = new PID(KpArm, KiArm, KdArm);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //HardwareDevice.Manufacturer;
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        climber = hardwareMap.get(DcMotor.class, "climber_motor");

        //Set default direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        climber.setDirection(DcMotor.Direction.FORWARD);

        tankDrive driveTrain = new tankDrive(rightDrive, leftDrive);

        arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        boolean armOn = true;

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

            //intake code
            if (gamepad1.left_bumper) {
                servo1.setPower(1);
                servo2.setPower(-1);
            } else if (gamepad1.left_stick_button) {
                servo1.setPower(-1);
                servo2.setPower(1);
            } else {
                servo1.setPower(0);
                servo2.setPower(0);
            }

            //climber code
            if (gamepad1.dpad_up) {
                climber.setPower(1);
                armOn = false;
            } else if (gamepad1.dpad_down) {
                climber.setPower(-1);
                armOn = false;
            } else {
                climber.setPower(0);
            }
            if (gamepad1.right_stick_button){
                armOn = true;
            }
            //armc ode
            arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

            int armEncoder = arm.getCurrentPosition();
            double armPower = arm_PID.runPID(armGoal, armEncoder);
            double test = arm.getCurrentPosition();

            if(gamepad1.y){
                armGoal = 850;
            }
            if(gamepad1.x){
                armGoal = 400;
            }
            if(gamepad1.b){
                armGoal = 1350;
            }
            if(gamepad1.a){
                armGoal = 1600;
            }

            if(gamepad1.start){
                arm.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
                armGoal = 0;
            }
            if (!gamepad1.right_stick_button && armOn) {
                arm.setPower(armPower);
            } else {
                arm.setPower(0);
            }

            armGoal = armGoal + (gamepad1.right_stick_y * 10);
            // Show the elapsed game time and wheel power.
            telemetry.addData("stuff", " armPower (%.2f), armEncoder (%.2f), Kp (%.2f, Goal (%.2f), Kd (%.2f), Ki (%.2f), error (%.2f)", armPower, test, Kp, targetAngle, Kd, Ki * 100000, targetAngle-angle);
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
