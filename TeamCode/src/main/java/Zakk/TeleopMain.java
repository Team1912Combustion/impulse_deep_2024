package Zakk;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "TeleOpMain", group = "Competition")
@Disabled
public class TeleopMain extends OpMode {

    /*
     * Declare Hardware
     */

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


    //    Intake

    CRServo  Arm;
    Servo  Claw;

    boolean ClawButtonLeft;
    boolean clawToggle1 = false;
    boolean ClawButtonRight;
    boolean clawToggle2 = false;


    @Override
    public void init() {

        // Initialize Wheels
        telemetry.addData("I", "Initializing Wheels");
        telemetry.update();

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

//        WheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        WheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        WheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Initialize Arm
        Arm = hardwareMap.get(CRServo.class, "Arm");
        Claw = hardwareMap.get(Servo.class, "Claw");


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
        float oneTriggerLeft = gamepad1.left_trigger;
        float oneTriggerRight = gamepad1.right_trigger;
        boolean oneBumperLeft = gamepad1.left_bumper;
        boolean oneBumperRight = gamepad1.right_bumper;
        boolean oneButtonX = gamepad1.x;

        // Gamepad 2
        double twoLeftStickYPower = -gamepad2.left_stick_y;
        double twoLeftStickXPower = gamepad2.left_stick_x;
        double twoRightStickXPower = gamepad2.right_stick_x;
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

        boolean firstTimeLeft = true;
        boolean firstTimeRight = true;
        boolean FirstTime = true;


        /*
         * Do Stuff Here!
         */

        MoveArmUp(oneTriggerRight);
        MoveArmDown(oneTriggerLeft);
        //Claw
        ClawButtonLeft = gamepad2.dpad_left;


        if (ClawButtonLeft == false && firstTimeLeft == false){
            firstTimeLeft = true;
        }

        if (ClawButtonLeft && firstTimeLeft){
            firstTimeLeft = false;
            clawToggle1 = !clawToggle1;
            if (clawToggle1){
                Claw.setPosition(0.46);
            } else {
                Claw.setPosition(0);
            }
        }

        ClawButtonRight = oneButtonX;



        telemetry.addData("Claw1", Claw.getPosition());

        ProMotorControl(oneLeftStickYPower, oneLeftStickXPower, oneRightStickXPower);
        ToggleSineDrive(oneButtonB);
        // Slow Controls
        ToggleSlowModeDrive(oneButtonA);

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


    //Intake

    private void MoveArmUp(float Zr){
        Arm.setPower(-Zr * 4);
        telemetry.addData("Drive Mode", Arm.getPower() + "% Power");
    }
    private void MoveArmDown(float Zl){
        Arm.setPower(Zl * 4);
        telemetry.addData("Drive Mode", Arm.getPower() + "% Power");
    }
    private void ClawOpen(boolean LeftBumper){

    }
    private void ClawClose(boolean RightBumper){

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


