package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Landing_Auto_Gyro", group="K9bot")
public class Landing_Auto_Gyro extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private DcMotor climber = null;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    static final double driveSpeed = Global_Variables.driveSpeed;
    static final double turnSpeed = Global_Variables.turnSpeed;
    static final double climbSpeed = Global_Variables.climbSpeed;
    static final int climbTime = Global_Variables.climbTime;

    @Override
    public void runOpMode() {

        // Stuff for hardware map
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm = hardwareMap.get(DcMotor.class, "arm_motor");
        climber = hardwareMap.get(DcMotor.class, "climber_motor");

        /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver"); //
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Release the velcro
        climber.setPower(-climbSpeed);
        sleep(climbTime*1000);
        climber.setPower(0);

        sleep(1000);

        //Drive backwards a little bit
        leftDrive.setPower(-driveSpeed);
        rightDrive.setPower(-driveSpeed);
        sleep(250);
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        //Turn to angle
        while (opModeIsActive()) {
            if(angles.firstAngle > 45){
                leftDrive.setPower(turnSpeed);
                rightDrive.setPower(-turnSpeed);
            }
            else if (angles.firstAngle < 50){
                leftDrive.setPower(-turnSpeed);
                rightDrive.setPower(turnSpeed);
            }
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            telemetry.update();
        }
    }
}
