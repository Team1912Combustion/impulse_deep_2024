package Zakk;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Adam.RevBlinkinLedDriver;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "HopeThisWorks", group = "Competition")
public class HopeThisWorks extends OpMode {
    /*
     * Declare Hardware
     */

    //IMU
    private IMU imu;

    // Wheels
    private DcMotor WheelFrontLeft;
    private DcMotor WheelFrontRight;
    private DcMotor WheelBackLeft;
    private DcMotor WheelBackRight;


    // SlowMode Drive
    private boolean slowModeDriveOn = true;
    private boolean buttonSlowDriveIsPressed = false;
    private final double SLOW_DRIVE = 0.4;
    private final double FAST_DRIVE = 1.0; //0.9;
    private double percentToSlowDrive = SLOW_DRIVE;

    // SineDrive
    private boolean sineDriveOn = true;
    private boolean buttonSineIsPressed = false;
    private double modifyBySine = Math.sin(Math.PI / 4);


    @Override
    public void init() {

        WheelFrontLeft = hardwareMap.dcMotor.get("WheelFL");
        WheelFrontRight = hardwareMap.dcMotor.get("WheelFR");
        WheelBackLeft = hardwareMap.dcMotor.get("WheelBL");
        WheelBackRight = hardwareMap.dcMotor.get("WheelBR");

        WheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Let the user know initialization is complete.
        telemetry.addData("I", "Initialization Complete! :D");
        telemetry.update();
    }



    @Override
    public void loop() {

        /*
         * Gamepad Controls
         */

        // Gamepad 1
        double oneLeftStickYPower = -gamepad1.left_stick_y;
        double oneLeftStickXPower = gamepad1.left_stick_x;
        double oneRightStickXPower = gamepad1.right_stick_x;
        boolean oneButtonA = gamepad1.a;
        boolean oneButtonB = gamepad1.b;
        boolean oneStart = gamepad1.start;

        // Gamepad 2
        boolean twoButtonA = gamepad2.a;
        boolean twoButtonB = gamepad2.b;
        boolean twoButtonX = gamepad2.x;
        boolean twoButtonY = gamepad2.y;
        boolean twoPadUp = gamepad2.dpad_up;
        boolean twoPadDown = gamepad2.dpad_down;
        boolean twoPadLeft = gamepad2.dpad_left;
        boolean twoPadRight = gamepad2.dpad_right;
        float twoTriggerLeft = gamepad2.left_trigger;
        float twoTriggerRight = gamepad2.right_trigger;
        boolean twoBumperLeft = gamepad2.left_bumper;
        boolean twoBumperRight = gamepad2.right_bumper;
        boolean twoBack = gamepad2.back;
        boolean twoStart = gamepad2.start;

        /*
         * Do Stuff Here!
         */


        // Slow Controls
        ToggleSlowModeDrive(oneButtonA);
        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);

        telemetry.update();


    }


    /*
     * Methods
     */

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
    //******************************************************************
    // Get the inputs from the controller for power [ PRO ]
    //******************************************************************
    private void ProMotorControl(double left_stick_y, double left_stick_x, double right_stick_x) {
        double powerLeftY = left_stick_y;   // DRIVE : Backward -1 <---> 1 Forward
        double powerLeftX = -left_stick_x * -1; // STRAFE:     Left -1 <---> 1 Right
        double powerRightX = right_stick_x; // ROTATE:     Left -1 <---> 1 Right

        double r = Math.hypot(powerLeftX, powerLeftY);
        double robotAngle = Math.atan2(powerLeftY, powerLeftX) - Math.PI / 4;
        double leftX = powerRightX;
        final double v1 = r * Math.cos(robotAngle) / modifyBySine + leftX;
        final double v2 = r * Math.sin(robotAngle) / modifyBySine - leftX;
        final double v3 = r * Math.sin(robotAngle) / modifyBySine + leftX;
        final double v4 = r * Math.cos(robotAngle) / modifyBySine - leftX;

        WheelFrontLeft.setPower(v1 * percentToSlowDrive);
        WheelFrontRight.setPower(v2 * percentToSlowDrive);
        WheelBackLeft.setPower(v3 * percentToSlowDrive);
        WheelBackRight.setPower(v4 * percentToSlowDrive);

        telemetry.addData("Wheel Front Left", v1 * percentToSlowDrive);
        telemetry.addData("Wheel Front Right", v2 * percentToSlowDrive);
        telemetry.addData("Wheel Back Left", v3 * percentToSlowDrive);
        telemetry.addData("Wheel Back Right", v4 * percentToSlowDrive);
    }

    private void PowerMotorControl(double FL,double BL, double FR, double BR){
        final double v1 = FL / modifyBySine;
        final double v2 = BL / modifyBySine;
        final double v3 = FR / modifyBySine;
        final double v4 = BR / modifyBySine;

        WheelFrontLeft.setPower(v1 * percentToSlowDrive);
        WheelFrontRight.setPower(v2 * percentToSlowDrive);
        WheelBackLeft.setPower(v3 * percentToSlowDrive);
        WheelBackRight.setPower(v4 * percentToSlowDrive);

        telemetry.addData("Wheel Front Left", v1 * percentToSlowDrive);
        telemetry.addData("Wheel Front Right", v2 * percentToSlowDrive);
        telemetry.addData("Wheel Back Left", v3 * percentToSlowDrive);
        telemetry.addData("Wheel Back Right", v4 * percentToSlowDrive);
    }




    private void ToggleSlowModeDrive(boolean button) {
        if (button && !buttonSlowDriveIsPressed) {
            buttonSlowDriveIsPressed = true;
            slowModeDriveOn = !slowModeDriveOn;
        }
        if (!button) {
            buttonSlowDriveIsPressed = false;
        }

        if (slowModeDriveOn) {
            percentToSlowDrive = SLOW_DRIVE;
            telemetry.addData("Drive Mode", "Slow: " + percentToSlowDrive + "% Power");
        } else {
            percentToSlowDrive = FAST_DRIVE;
            telemetry.addData("Drive Mode", "Fast: " + percentToSlowDrive + "% Power");
        }
    }


    private void ToggleSineDrive(boolean button) {
        if (button && !buttonSineIsPressed) {
            buttonSineIsPressed = true;
            sineDriveOn = !sineDriveOn;
        }
        if (!button) {
            buttonSineIsPressed = false;
        }

        if (sineDriveOn) {
            modifyBySine = Math.sin(Math.PI / 4);
            telemetry.addData("Sine Drive", "ON");
        } else {
            modifyBySine = 1;
            telemetry.addData("Sine Drive", "OFF");
        }
    }
}


