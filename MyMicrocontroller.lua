--- Developed using LifeBoatAPI - Stormworks Lua plugin for VSCode - https://code.visualstudio.com/download (search "Stormworks Lua with LifeboatAPI" extension)
--- If you have any issues, please report them here: https://github.com/nameouschangey/STORMWORKS_VSCodeExtension/issues - by Nameous Changey


--[====[ HOTKEYS ]====]
-- Press F6 to simulate this file
-- Press F7 to build the project, copy the output from /_build/out/ into the game to use
-- Remember to set your Author name etc. in the settings: CTRL+COMMA


--[====[ EDITABLE SIMULATOR CONFIG - *automatically removed from the F7 build output ]====]
---@section __LB_SIMULATOR_ONLY__
do
    ---@type Simulator -- Set properties and screen sizes here - will run once when the script is loaded
    simulator = simulator
    simulator:setScreen(1, "3x3")
    simulator:setProperty("ExampleNumberProperty", 123)

    -- Runs every tick just before onTick; allows you to simulate the inputs changing
    ---@param simulator Simulator Use simulator:<function>() to set inputs etc.
    ---@param ticks     number Number of ticks since simulator started
    function onLBSimulatorTick(simulator, ticks)

        -- touchscreen defaults
        local screenConnection = simulator:getTouchScreen(1)
        simulator:setInputBool(1, screenConnection.isTouched)
        simulator:setInputNumber(1, screenConnection.width)
        simulator:setInputNumber(2, screenConnection.height)
        simulator:setInputNumber(3, screenConnection.touchX)
        simulator:setInputNumber(4, screenConnection.touchY)

        -- NEW! button/slider options from the UI
        simulator:setInputBool(31, simulator:getIsClicked(1))       -- if button 1 is clicked, provide an ON pulse for input.getBool(31)
        simulator:setInputNumber(31, simulator:getSlider(1))        -- set input 31 to the value of slider 1

        simulator:setInputBool(32, simulator:getIsToggled(2))       -- make button 2 a toggle, for input.getBool(32)
        simulator:setInputNumber(32, simulator:getSlider(2) * 50)   -- set input 32 to the value from slider 2 * 50
    end;
end
---@endsection


--[====[ IN-GAME CODE ]====]

-- try require("Folder.Filename") to include code from another file in this, so you can store code in libraries
-- the "LifeBoatAPI" is included by default in /_build/libs/ - you can use require("LifeBoatAPI") to get this, and use all the LifeBoatAPI.<functions>!

ticks = 0

sweepRight = true --global boolean to determine the direction of the laser sweep
laserSweepSpeed = 1 --global variable to determine the speed of the laser sweep
laserSweepAngle = 0 --global variable to store the current angle of the laser sweep



function onTick()-- the main function that runs every tick

    ticks = ticks + 1 -- increment the tick counter

    -- Retrieve inputs from the microcontroller and store them in variables

    scriptIsEnabled = input.getBool(1) --boolean number 1 will be the on/off switch for the script
    currentAzimuth = input.getNumber(17) * 2 * math.pi --retrieve the azimuth in turns and convert to radians
    laserDistanceReading = input.getNumber(18) --retrieve the distance reading from the laser
    gpsTargetX = input.getNumber(19) --retrieve the target x coordinate for navigation
    gpsTargetY = input.getNumber(20) --retrieve the target y coordinate for navigation
    currentGPSPositionX = input.getNumber(1) --retrieve the current x coordinate from the GPS
    currentGPSPositionY = input.getNumber(3) --retrieve the current y coordinate from the GPS
    currentPitch = input.getNumber(15) * 2 * math.pi --retrieve the pitch in turns and convert to radians
    currentRoll = -input.getNumber(16) * 2 * math.pi --retrieve the roll in turns and convert to radians
    
    --when the inpout is on, go through the calculations
    if scriptIsEnabled then
        laserSweep() --run the laser sweep function
    end


end


--this function produces the values required to allow the laser to sweep left and right in line with the horizon
function laserSweep()

    --this section of code changes the laser angle as to make it sweep left and right

    if sweepRight == true then --if the sweep is going right, add to the angle
        laserSweepAngle = laserSweepAngle + (0.00125 * laserSweepSpeed)
    else --if the sweep is going left, subtract from the angle
        laserSweepAngle = laserSweepAngle - (0.00125 * laserSweepSpeed)
    end

    if laserSweepAngle >= 0.125 then --if the angle is greater than 0.125, change the direction of the sweep
        sweepRight = false

    elseif laserSweepAngle <= -0.125 then --if the angle is less than -0.125, change the direction of the sweep
        sweepRight = true
    end

    --this section of the code takes pitch and roll readings and adjusts the laser angle to keep it level

    laserSweepAngleInRadians = laserSweepAngle * 2 * math.pi

    --we imagine the place the laser is pointing as a vector in 3D space
    
    X = 0
    Y = 0
    Z = 1

    --we then rotate this vector in the x and y direction based on the pitch and roll of the vehicle, as well as the desired sweep angle

    --y axis rotation
    X1 = (Z * math.sin(laserSweepAngleInRadians))
    Y1 = Y
    Z1 = (Z * math.cos(laserSweepAngleInRadians))

    --x axis rotation
    X2 = X1
    Y2 = - (Z1 * math.sin(currentPitch))
    Z2 =  (Z1 * math.cos(currentPitch))

    X6 = 1
    Y6 = 0
    Z6 = 0

    --Z axis rotation
    X3 = (X6*math.cos(currentRoll)) 
    Y3 = (X6 * math.sin(currentRoll)) 
    Z3 = Z6

    -- i honestly cant remember exactly what all this math does, but it works so im not going to touch it

    --calculatte new rotation

    currentRoll = math.atan(Y3,X3)

    --Z axis rotation
    X5 = (X2*math.cos(currentRoll)) - (Y2 * math.sin(currentRoll))
    Y5 = (X2 * math.sin(currentRoll)) + (Y2 * math.cos(currentRoll))
    Z5 = Z2
    

    yaw = math.atan(X5,Z5)

    output.setNumber(1,8 * yaw/(2*math.pi)) -- output the pitch value for the laser
    
    -- as the way the laser angle is controlled works a bit funny, we need to apply another rotation to make it behave as expected

    --y axis rotation
    X4 = (Z5 * math.sin(-yaw)) + (X5 * math.cos(-yaw))
    Y4 = Y5
    Z4 = (Z5 * math.cos(-yaw)) - (X5 * math.sin(-yaw))

    output.setNumber(2,8 * math.atan(Y4,Z4)/(2*math.pi)) -- output the yaw value for the laser

end

function onDraw()
    screen.drawCircle(16,16,5)
end



