# Status constants
# SEARCHING - The camera is searching for a previously known position to locate itself
# OK - Positional tracking is working normally
# OFF - Positional tracking is not enabled.
# FPS_TOO_LOW - Effective FPS is too low to give proper results for motion tracking. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720))
# SEARCHING_FLOOR_PLANE - The camera is searching for the floor plane to locate itself related to it, the REFERENCE_FRAME::WORLD will be set afterward.
uint8 SEARCHING=0 
uint8 OK = 1
uint8 OFF = 2
uint8 FPS_TOO_LOW = 3
uint8 SEARCHING_FLOOR_PLANE = 3

# Status
uint8 status