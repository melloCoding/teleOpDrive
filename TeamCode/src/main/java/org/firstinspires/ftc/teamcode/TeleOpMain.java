//Import required librarys
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "TeleOpMain", group = "Drive")
public class TeleOpMain extends LinearOpMode {

    //Power Constant
    private static final double powerConstant = .51;
    double voltage = 0;
    double speed = 1.5;
    //battery Constant
    double batteryConst = 13.5;
    long debounceInterval = 100;
    //variables for claw states
    double clawOpen = 0.7;
    int clawClose = 0;
    int clawReset = 1;
    int armStartPos = -2; //The value for the arm rotation position to move it back to the start.
    int armMiddlePos = -330; //The value for the arm rotation position to move it to straight up and down
    int armBottomPos = -760; //The value for the arm rotation position to move it to pick things up off the floor
    int clawHeightBottom = 0;
    int clawHeightMid = 3312;
    int clawHeightHigh = 7312;
    double stdSpeedMulti = 1.5;
    double slowSpeedMulti = 0.5;
    double wristLoc = 1;
    double armLoc = 0;
    //Drive Wheels
    private DcMotor FLMoto;
    private DcMotor FRMoto;
    private DcMotor BLMoto;
    private DcMotor BRMoto;
    //arm
    private DcMotor arm;
    private DcMotor string;
    private Servo claw;
    private Servo wrist;
    private Servo Finger;
    private CRServo tapeServo;
    private DcMotor tapeMoto;
    private DcMotor tapeRoto;
    //voltage sensor
    private VoltageSensor VoltSens;
    // ColorSensor/Distande Sensor sampleSen;
    private DistanceSensor distance;

  /*
  Where the motors are plugged into
  Control Hub:
  BRMoto - Motor Port 0
  BLMoto - Motor Port 1
  FLMoto - Motor Port 2
  FRMoto - Motor Port 3

  Expansion Hub:
  string - Motor Port 1
  arm - Motor Port 2
  wrist - Servo Port 1
  claw - Servo Port 0
  */

    //BlinkinLEDs
    //private RevBlinkinLedDriver blinkinLedDriver;
    //private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    //private RevBlinkinLedDriver.BlinkinPattern StopPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    private NormalizedColorSensor color;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double wUP = .3;
        double Iin = 1;
        double Fly = 1;

        //****************************************//
        // Map all robot hardware				  //
        //****************************************//

        //Main Drive Motors
        //****************************************//
        //BR means "back right"				   	  //
        //BL means "back left"					  //
        //FR means "front right"				  //
        //FL means "front left"					  //
        //****************************************//
        FLMoto = hardwareMap.dcMotor.get("FLMoto"); //Front left drive motor
        FRMoto = hardwareMap.dcMotor.get("BRMoto"); //Front right drive motor
        BLMoto = hardwareMap.dcMotor.get("BLMoto");
        BRMoto = hardwareMap.dcMotor.get("FRMoto");


        // Map Sensors
        distance = hardwareMap.get(DistanceSensor.class, "colorRange");
        color = hardwareMap.get(NormalizedColorSensor.class, "colorRange");


        //BlinkinLEDs
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        //blinkinLedDriver.setPattern(BasePattern);

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.

        float[] hsvValues = {0F, 0F, 0F};

        final float[] values = hsvValues;

        double contPower = 0.0;

        //*****************************************//
        // Put initialization blocks here.			 //
        //*****************************************//

        //***************************************************//
        // Set direction of the main drive motors				   //
        //***************************************************//
        FLMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        FRMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        BRMoto.setDirection(DcMotorSimple.Direction.REVERSE);


        //Set the zero power behavor of the main drive motors to FLOAT
        FLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 2;

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (color instanceof SwitchableLight) {
            ((SwitchableLight) color).enableLight(true);
        }

        //The waitForStart() function will wait for the start button will begin
        waitForStart();

        //******************************************//
        // Run code while op mode is active			//
        //******************************************//
        while (opModeIsActive()) {
            //**************************************************************//
            //Telemetry Code the stuff that appears on the right side of the//
            //driver hub													//
            //**************************************************************//
            telemetry.addData("Status", "opModeIsActive");

            //comented as the are unneeded for usual oporation
	  /*
		telemetry.addData("Arm Rotation:", arm.getCurrentPosition());
		telemetry.addData("Arm Height:", string.getCurrentPosition());
		*/

	  /*
		// color sensor info
		// Get the normalized colors from the sensor
	  NormalizedRGBA colors = color.getNormalizedColors();
	*/

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            // Update the hsvValues array by passing it to Color.colorToHSV()
	  /*
	  Color.colorToHSV(colors.toColor(), hsvValues);

	  telemetry.addLine()
			  .addData("Red", "%.3f", colors.red)
			  .addData("Green", "%.3f", colors.green)
			  .addData("Blue", "%.3f", colors.blue);
	  telemetry.addLine()
			  .addData("Hue", "%.3f", hsvValues[0])
			  .addData("Saturation", "%.3f", hsvValues[1])
			  .addData("Value", "%.3f", hsvValues[2]);
	  telemetry.addData("Alpha", "%.3f", colors.alpha);

		*/
            // generic DistanceSensor methods.
            //comented as the are unneeded for usual oporation
            // telemetry.addData("deviceName", distance.getDeviceName() );
            // telemetry.addData("range", String.format("%.01f mm", distance.getDistance(DistanceUnit.MM)));
            // telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
            // telemetry.addData("range", String.format("%.01f m", distance.getDistance(DistanceUnit.METER)));
            // telemetry.addData("range", String.format("%.01f in", distance.getDistance(DistanceUnit.INCH)));

            telemetry.update();

            //Gamepad1

            //slow mode
            //changes the speed variable to 0.5 as long as the the a button is pressed
            if (gamepad1.right_bumper) {
                speed = slowSpeedMulti; //change to slow mode speed
            } else {
                speed = stdSpeedMulti; //change speed back to normal speed
            }

            //regular drive controls
            //sets the drive power to gamepad1 left_stick_y for the left drive train
            //and gamepad1 right_stick_y for the right side drive train then it sets it
            //all to be multiplied by the speed modifier
            FRMoto.setPower(gamepad1.right_stick_y * speed);
            FLMoto.setPower(gamepad1.left_stick_y * speed);
            BRMoto.setPower(gamepad1.right_stick_y * speed);
            BLMoto.setPower(gamepad1.left_stick_y * speed);

            //Strafe Right using gamepad1 right_trigger
            FRMoto.setPower(-gamepad1.right_trigger);
            FLMoto.setPower(-gamepad1.right_trigger);
            BRMoto.setPower(gamepad1.right_trigger);
            BLMoto.setPower(gamepad1.right_trigger);

            // Strafe Left using gamepad1 left_trigger
            FRMoto.setPower(gamepad1.left_trigger);
            FLMoto.setPower(gamepad1.left_trigger);
            BRMoto.setPower(-gamepad1.left_trigger);
            BLMoto.setPower(-gamepad1.left_trigger);
        }

    }

    protected enum DisplayKind {
        MANUAL,
        AUTO,
    }
  /*
  void motorToPosition(namespace selectMotor, float setMotorPower, float motorTargetPositon){
	  selectMotor.setTargetPosition(motorTargetPositon);
	  selectMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	  selectMotor.setPower(setMotorPower);
	}
	*/
}
