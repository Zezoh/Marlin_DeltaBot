#if ENABLED(DELTA)

  // Make delta curves from many straight lines (linear interpolation).
  // This is a trade-off between visible corners (not enough segments)
  // and processor overload (too many expensive sqrt calls).
  #define DELTA_SEGMENTS_PER_SECOND 80

  // Limit segment granularity to reduce planner load.
  #define KINEMATIC_SEGMENT_MIN_LENGTH 0.25 // mm

  //Fast inverse sqrt from Quake III Arena                                                
  //See: https://en.wikipedia.org/wiki/Fast_inverse_square_root                                                                                                               
  #define DELTA_FAST_SQRT

  // Convert feedrates to apply to the Effector instead of the Carriages
  //#define DELTA_FEEDRATE_SCALING

  // Optional tower-motion cornering slowdown for smoother delta movement
  //#define DELTA_MOTION_OPTIMIZATION

  // After homing move down to a height where XY movement is unconstrained
  //#define DELTA_HOME_TO_SAFE_ZONE

  // Delta calibration menu
  // uncomment to add three points calibration menu option.
  // See http://minow.blogspot.com/index.html#4918805519571907051
  //#define DELTA_CALIBRATION_MENU

  // uncomment to add G33 Delta Auto-Calibration (Enable EEPROM_SETTINGS to store results)
  #define DELTA_AUTO_CALIBRATION

  // NOTE NB all values for DELTA_* values MUST be floating point, so always have a decimal point in them

  #if ENABLED(DELTA_AUTO_CALIBRATION)
    // set the default number of probe points : n*n (1 -> 7)
    #define DELTA_CALIBRATION_DEFAULT_POINTS 3 // 4
  #endif

  #if ENABLED(DELTA_AUTO_CALIBRATION) || ENABLED(DELTA_CALIBRATION_MENU)
    // Set the radius for the calibration probe points - max DELTA_PRINTABLE_RADIUS for non-eccentric probes
    #define DELTA_CALIBRATION_RADIUS (DELTA_PRINTABLE_RADIUS - 15.0) // mm
    // Set the steprate for papertest probing
    #define PROBE_MANUALLY_STEP 0.05 // mm
  #endif

  // Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
  #define DELTA_PRINTABLE_RADIUS 85.0 //100.0 // mm

  // Center-to-center distance of the holes in the diagonal push rods.
  #define DELTA_DIAGONAL_ROD 210.00 //239.50 // mm

  // height from z=0 to home position
  #define DELTA_HEIGHT 225.00 // get this value from auto calibrate

  #define DELTA_ENDSTOP_ADJ { 0.0, 0.0, 0.0 } // get these from auto calibrate

  // Horizontal distance bridged by diagonal push rods when effector is centered.
  #define DELTA_RADIUS 90.0 //mm  Get this value from auto calibrate

  // Trim adjustments for individual towers
  // tower angle corrections for X and Y tower / rotate XYZ so Z tower angle = 0
  // measured in degrees anticlockwise looking from above the printer
  #define DELTA_TOWER_ANGLE_TRIM { 0.0, 0.0, 0.0 } // get these values from auto calibrate

  // delta radius and diaginal rod adjustments measured in mm
  #define DELTA_RADIUS_TRIM_TOWER { 0.0, 0.0, 0.0 }
  //#define DELTA_DIAGONAL_ROD_TRIM_TOWER { 60.0, 60.0, 60.0 }

  // ISSUE #6 FIX: Configurable FSR Calibration Height - prevents hardcoded 66.90 limit
  #define FSR_CALIBRATION_MAX_Z (DELTA_HEIGHT - 2.0)

#endif
