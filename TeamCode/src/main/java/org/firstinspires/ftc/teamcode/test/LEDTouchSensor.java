package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver.LayerHeight;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;
import java.util.ArrayList;

@TeleOp(name = "LED Touch Sensor - Multi Pattern", group = "Test")
public class LEDTouchSensor extends LinearOpMode {

    public GoBildaPrismDriver prism;
    public TouchSensor touchSensor;

    // A list of code actions to run when switching patterns
    private final ArrayList<Runnable> patternActions = new ArrayList<>();
    private int currentPatternIndex = 0;
    private boolean lastTouchState = false;

    @Override
    public void runOpMode() {
        // ==========================================
        // HARDWARE
        // ==========================================
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        // ==========================================
        // INITIALIZE ALL ANIMATIONS
        // ==========================================
        PrismAnimations.Solid solidGreen   = new PrismAnimations.Solid(Color.GREEN);
        PrismAnimations.Solid solidPurple  = new PrismAnimations.Solid(Color.PURPLE);
        PrismAnimations.Solid solidPink    = new PrismAnimations.Solid(Color.PINK);
        PrismAnimations.Solid solidMagenta = new PrismAnimations.Solid(Color.MAGENTA);
        PrismAnimations.Solid solidCyan    = new PrismAnimations.Solid(Color.CYAN);
        PrismAnimations.Solid solidTeal    = new PrismAnimations.Solid(Color.TEAL);
        PrismAnimations.Solid solidOrange  = new PrismAnimations.Solid(Color.ORANGE);
        PrismAnimations.Solid solidRed     = new PrismAnimations.Solid(Color.RED);
        PrismAnimations.Solid solidYellow  = new PrismAnimations.Solid(Color.YELLOW);
        PrismAnimations.Solid solidOlive   = new PrismAnimations.Solid(Color.OLIVE);
        PrismAnimations.Solid solidBlue    = new PrismAnimations.Solid(Color.BLUE);
        PrismAnimations.Solid solidWhite   = new PrismAnimations.Solid(Color.WHITE);

        PrismAnimations.Rainbow rainbow   = new PrismAnimations.Rainbow();

        // Configure default layout properties for all items
        int brightness = 50;
        int startIdx = 0;
        int stopIdx = 12;

        solidGreen.setBrightness(brightness);   solidGreen.setStartIndex(startIdx);   solidGreen.setStopIndex(stopIdx);
        solidPurple.setBrightness(brightness);  solidPurple.setStartIndex(startIdx);  solidPurple.setStopIndex(stopIdx);
        solidPink.setBrightness(brightness);    solidPink.setStartIndex(startIdx);    solidPink.setStopIndex(stopIdx);
        solidMagenta.setBrightness(brightness); solidMagenta.setStartIndex(startIdx); solidMagenta.setStopIndex(stopIdx);
        solidCyan.setBrightness(brightness);    solidCyan.setStartIndex(startIdx);    solidCyan.setStopIndex(stopIdx);
        solidTeal.setBrightness(brightness);    solidTeal.setStartIndex(startIdx);    solidTeal.setStopIndex(stopIdx);
        solidOrange.setBrightness(brightness);  solidOrange.setStartIndex(startIdx);  solidOrange.setStopIndex(stopIdx);
        solidRed.setBrightness(brightness);     solidRed.setStartIndex(startIdx);     solidRed.setStopIndex(stopIdx);
        solidYellow.setBrightness(brightness);  solidYellow.setStartIndex(startIdx);  solidYellow.setStopIndex(stopIdx);
        solidOlive.setBrightness(brightness);   solidOlive.setStartIndex(startIdx);   solidOlive.setStopIndex(stopIdx);
        solidBlue.setBrightness(brightness);    solidBlue.setStartIndex(startIdx);    solidBlue.setStopIndex(stopIdx);
        solidWhite.setBrightness(brightness);   solidWhite.setStartIndex(startIdx);   solidWhite.setStopIndex(stopIdx);

        rainbow.setBrightness(brightness);      rainbow.setStartIndex(startIdx);      rainbow.setStopIndex(stopIdx);

        // ==========================================
        // MAP PATTERNS TO ACTIONS
        // ==========================================
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidGreen));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidPurple));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidPink));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidMagenta));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidCyan));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidTeal));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidOrange));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidRed));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidYellow));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidOlive));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidBlue));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidWhite));
        patternActions.add(() -> prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, rainbow));

        // ==========================================
        // INIT TELEMETRY
        // ==========================================
        telemetry.addLine("PRISM LED MULTI-PATTERN TEST");
        telemetry.addLine("----------------------------");
        telemetry.addData("Total Patterns Loaded", patternActions.size());
        telemetry.addLine("Press START...");
        telemetry.update();

        // ==========================================
        // WAIT FOR START
        // ==========================================
        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // Run the very first action (Green) before entering the loop
        patternActions.get(currentPatternIndex).run();

        // ==========================================
        // MAIN LOOP
        // ==========================================
        while (opModeIsActive()) {
            boolean currentTouchState = touchSensor.isPressed();

            // ==========================================
            // DETECT BUTTON CLICK (Edge Detection)
            // ==========================================
            if (currentTouchState && !lastTouchState) {
                currentPatternIndex++;

                if (currentPatternIndex >= patternActions.size()) {
                    currentPatternIndex = 0;
                }

                // Fire the action block for the new pattern index
                patternActions.get(currentPatternIndex).run();
            }

            lastTouchState = currentTouchState;

            // ==========================================
            // TELEMETRY
            // ==========================================
            telemetry.addLine("PRISM LED MULTI-PATTERN TEST");
            telemetry.addLine("----------------------------");
            telemetry.addData("Touch Sensor", currentTouchState ? "PRESSED" : "RELEASED");
            telemetry.addData("Active Pattern ID", currentPatternIndex);
            telemetry.addData("FPS", prism.getCurrentFPS());
            telemetry.update();

            sleep(20);
        }
    }
}
