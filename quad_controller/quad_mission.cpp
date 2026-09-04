//Top level quad state machine
// Call this for all your quad needs!
// Made and Upkept by Darcy Moyes - 25280025
// last patch was 4 September 26

// -- Quadcopter Sates --
// IDLE          = Motors are idle, here when waiting for a command
// TAKEOFF       = Start to move to a coasting height
// SEARCH        = Following path, looking for mushrooms, avoid obstacles -- the big one
// RETURN        = Returning home to start point - doesnt search in this mode
// LAND          = self explanatory - lands the quadcopter
//GROUNDED       = on the ground - can lead to takeoff



