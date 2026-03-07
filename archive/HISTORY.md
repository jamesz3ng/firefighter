# Development History

This project was developed over 6 weeks (April 28 - June 9, 2025) without consistent access to git version control. Most iterations were saved by copying the working directory to a new folder, resulting in the 15 snapshots preserved here.

## Timeline

### 01 - first_code (Apr 28 - May 20)
The original codebase. Basic obstacle avoidance with states: INITIALISING, DETECT_FIRE, AVOID, EXTINGUISH, CHECK_GYRO, STOPPED, LEFT, RIGHT. Also contains `search_fire.ino`, a standalone fire-searching module using servo-mounted phototransistors.

### 02 - SearchFire (May 27)
Promoted standalone copy of `search_fire.ino`. States: INITIALISING, DETECT_FIRE, DRIVE_TO_FIRE, STOPPED. Pure fire-tracking logic with dual phototransistors (A9/A10).

### 03 - first_code_may_23 (May 23)
Added `#define` macros for servo pin, phototransistor pins, and threshold constants. First use of named constants instead of magic numbers.

### 04 - merged (May 30)
First attempt at merging obstacle avoidance with fire detection. Added DRIVE_TO_FIRE state to the original state machine.

### 05 - MergedFirefighter (May 30)
Second merge attempt. Refined state machine with DRIVE_TO_FIRE repositioned in the enum.

### 06 - manual_merge (May 30-31)
Hand-merged version combining fire search + obstacle avoidance logic. Removed STARTUP_DELAY define. Added extinguish function.

### 07 - 31_05_morning_session (May 31)
Significant size jump (+116 lines). Developed on a cloud IDE (Eclipse Theia). Continued integration of fire tracking with navigation.

### 08 - 01_06_jinsol (Jun 1)
Pair-programming session. Replaced AVOID state with NAVIGATE. Added Kalman filtering for phototransistor readings.

### 09 - RevertingBack (Jun 1)
Reverted the NAVIGATE rename back to AVOID after testing. Kept Kalman filter additions.

### 10 - currentlyworkingon (Jun 1)
Active development snapshot. Continued refinement of obstacle avoidance + fire tracking integration.

### 11 - WorkingExceptThreshhold (Jun 1-2)
Fire detection thresholds not yet tuned. Kalman filtering integrated for phototransistors. "Working except threshold" -- close but not reliably detecting fire.

### 12 - AlmostWorking (Jun 3-4)
Experimented with 4 phototransistors (2 per side: A9/A11 left, A10/A12 right) for better directional fire detection. Reverted to 2 in later iterations.

### 13 - sometingwong (Jun 4-5)
Debug session. Bluetooth serial commented out (was causing serial conflicts). Back to single phototransistor per side. Milestone: successfully finds fire after avoiding obstacle.

### 14 - finalday.ino (Jun 5)
Final day push. Bluetooth still disabled. Refinements to motor power and detection thresholds.

### 15 - last_gooning.ino (Jun 9)
**The final working version that achieved full marks.** This is the same code as `src/firefighter/firefighter.ino` (renamed). Complete state machine with reliable fire detection, proportional steering toward fire, obstacle avoidance, and fan-based extinguishing for 2 fires.
