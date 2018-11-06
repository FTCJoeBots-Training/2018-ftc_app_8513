package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2018 JoeBots FTC Rover Ruckus challenge. This file has been generalized to work as a base for
 * all three JoeBots FTC teams (8513, 11855, and 13702). As the season progresses, this file may be
 * customized for each individual team in their own branch.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor1 (left front)
 * motor2 (right front)
 * motor3 (left rear)
 * motor4 (right rear)
 * imu - navigation features
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class AshtonHardwareJoeBot2018
{
    /* Public OpMode members. */

    // Declare Motors
    public DcMotor  motor1 = null; // Left Front
    public DcMotor  motor2 = null; // Right Front
    public DcMotor  motor3 = null; // Left Rear
    public DcMotor  motor4 = null; // Right Rear

    // Declare Sensors
    public BNO055IMU imu;                  // The IMU sensor object
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;


    // Variables used for IMU tracking...
    public Orientation angles;
    public Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation lastImuAngles = new Orientation();
    private double globalAngle;

    // Declare Static members for calculations
    static final double COUNTS_PER_MOTOR_REV    = 1120;
    static final double DRIVE_GEAR_REDUCTION    = 1;
    static final double WHEEL_DIAMETER_INCHES   = 4.0;
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);


    /* Constructor */
    public AshtonHardwareJoeBot2018(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        motor1 = hwMap.dcMotor.get("motor0");
        motor2 = hwMap.dcMotor.get("motor1");
        motor3 = hwMap.dcMotor.get("motor2");
        motor4 = hwMap.dcMotor.get("motor3");

        // Set Default Motor Directions
        motor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor2.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        motor3.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motor4.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // IMU Initializaiton
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        motor1.setMode(mode);
        motor2.setMode(mode);
        motor3.setMode(mode);
        motor4.setMode(mode);
    }

    /**
     *
     * void moveRobot(double forward, double right, double clockwise)
     *
     * Calculates power settings for Mecanum drive for JoeBots
     *
     * @param forward
     * @param right
     * @param clockwise
     *
     */
    public void moveRobot(double forward, double right, double clockwise) {

        // Declare Variables to hold calculated power values for each motor
        double power1;
        double power2;
        double power3;
        double power4;
        double max;

        power1 = forward + clockwise + right;
        power2 = forward - clockwise - right;
        power3 = forward + clockwise - right;
        power4 = forward - clockwise + right;

        // Normalize Wheel speeds so that no speed exceeds 1.0
        max = Math.abs(power1);
        if (Math.abs(power2) > max) {
            max = Math.abs(power2);
        }
        if (Math.abs(power3) > max) {
            max = Math.abs(power3);
        }
        if (Math.abs(power4) > max) {
            max = Math.abs(power4);
        }

        if (max > 1) {
            power1 /= max;
            power2 /= max;
            power3 /= max;
            power4 /= max;
        }

        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);
        motor4.setPower(power4);

    }

    /**
     *
     * stop()
     *
     * method to set all motor powers to zero
     *
     */

    public void stop() {

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

    }

    /**
     *
     * moveInches(double inches, double power)
     *
     * method to drive forward (only) for a set # of inches at a set power
     *
     * @param inches
     * @param power
     * @param timeoutSec
     *
     */

    public void moveInches(double inches, double power, int timeoutSec) {

        // method will accept a value in inches and a power setting and calculate the number of
        // rotations required to drive the given distance.

        // Tell Telemetry what we're starting
        myOpMode.telemetry.log().add("Starting moveInches method");

        // Declare needed variables
        int newMotor1Target;
        int newMotor2Target;
        int newMotor3Target;
        int newMotor4Target;

        // Check to make sure the OpMode is still active; If it isn't don't run the method
        if(myOpMode.opModeIsActive()) {

            // Determine new target positions for each wheel
            newMotor1Target = motor1.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newMotor2Target = motor2.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newMotor3Target = motor3.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newMotor4Target = motor4.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            // Send target Positions to motors
            motor1.setTargetPosition(newMotor1Target);
            motor2.setTargetPosition(newMotor2Target);
            motor3.setTargetPosition(newMotor3Target);
            motor4.setTargetPosition(newMotor4Target);

            // Set Robot to RUN_TO_POSITION mode
            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the runtime
            runtime.reset();

            // Start moving the robot
            moveRobot(power,0,0);

            // Keep looping (wait) until the motors are finished or timeout is reached.
            while (myOpMode.opModeIsActive() && (runtime.seconds() < timeoutSec) &&
                    (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy())) {


                //Compose Telemetry message
                myOpMode.telemetry.addLine("> Waiting for robot to reach target");
                myOpMode.telemetry.addLine("Curr. Pos. |")
                        .addData("1:",motor1.getCurrentPosition())
                        .addData("2:",motor2.getCurrentPosition())
                        .addData("3:",motor3.getCurrentPosition())
                        .addData("4:",motor4.getCurrentPosition());
                myOpMode.telemetry.addLine("Target | ")
                        .addData("1:",newMotor1Target)
                        .addData("2:",newMotor2Target)
                        .addData("3:",newMotor3Target)
                        .addData("4:",newMotor4Target);
                myOpMode.telemetry.addData("Power: ", power);
                myOpMode.telemetry.update();

                myOpMode.idle();
            }

            // Stop the motors
            stop();

            // Update telemetry log
            myOpMode.telemetry.log().add("Ending moveInches method");

            // Set the motors back to standard mode
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    /**
     *
     * resetImuAngle()
     *
     * Method to grab the current reading from the IMU and set the cumulative angle tracking
     * to 0
     *
     */

    private void resetAngle(){

        // Grab reading from IMU and store it in lastImuAngles
        lastImuAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Set globalAngle to zero
        globalAngle = 0;

    }

    /**
     *
     * getAngle()
     *
     * Gets the current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees (+ left, - right)
     *
     */

    private double getAngle(){

        // Grab the current IMU Angle reading
        Orientation currAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

        // Determine the difference between the current Angle reading and the last reset
        double deltaAngle = currAngles.firstAngle - lastImuAngles.firstAngle;
        deltaAngle=-deltaAngle;

        // Since the Rev IMU measures in Euler angles (-180 <-> +180), we need to detect this
        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastImuAngles = currAngles;

        return globalAngle;

    }

    /**
     *
     * rotate(int degrees, double power)
     *
     * Does not support turning more than 180 degrees.
     *
     * @param degrees
     * @param power
     *
     *
     */

    public void rotate(int degrees, double power){

        myOpMode.telemetry.log().add("Starting rotate method");

        // Restart IMU movement tracking
        resetAngle();

        // getAngle returns + when rotating counter clockwise and - when rotating clockwise
        // set power (speed) negative when turning right
        if (degrees < 0 ) power = -power;

        // start robot turning
        moveRobot(0,0,power);

        // stop turning when getAngle() returns a value greater or less than intended degrees
        if (degrees < 0) {
            // Expect this to be a left turn
            // On a left turn, since we start on zero, we have to get off zero first

            while (myOpMode.opModeIsActive() && getAngle() == 0) {
                myOpMode.telemetry.addLine(">getAngle() returned 0");
                myOpMode.telemetry.addLine(">>")
                        .addData("Cur: ", getAngle())
                        .addData("Tar: ", degrees);
                myOpMode.telemetry.update();
            }

            while (myOpMode.opModeIsActive() && getAngle() > degrees) {
                myOpMode.telemetry.addLine(">getAngle() returned >0");
                myOpMode.telemetry.addLine(">>")
                        .addData("Cur: ", getAngle())
                        .addData("Tar: ", degrees);
                myOpMode.telemetry.update();
            }
        } else {
            // left turn

            while (myOpMode.opModeIsActive() && getAngle() < degrees) {
                myOpMode.telemetry.addLine(">getAngle() returned <0");
                myOpMode.telemetry.addLine(">>")
                        .addData("Cur: ", getAngle())
                        .addData("Tar: ", degrees);
                myOpMode.telemetry.update();
            }

        }


        //Stop the motors
        stop();

        // reset IMU tracking
        resetAngle();



    }

    public boolean isItGold(){
        //Check for gold mineral (yellow)

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        if (hsvValues[0] >= 20 && hsvValues [0] <= 40) {
            return true;
        }
        else {
            return false;
        }

    }

}






