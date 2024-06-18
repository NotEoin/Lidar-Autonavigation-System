--- Developed using LifeBoatAPI - Stormworks Lua plugin for VSCode - https://code.visualstudio.com/download (search "Stormworks Lua with LifeboatAPI" extension)
--- If you have any issues, please report them here: https://github.com/nameouschangey/STORMWORKS_VSCodeExtension/issues - by Nameous Changey


--[====[ HOTKEYS ]====]
-- Press F6 to simulate this file
-- Press F7 to build the project, copy the output from /_build/out/ into the game to use
-- Remember to set your Author name etc. in the settings: CTRL+COMMA


--[====[ EDITABLE SIMULATOR CONFIG - *automatically removed from the F7 build output ]====]
---@section __LB_SIMULATOR_ONLY__

xPosition = 0
yPosition = 0

do
    ---@type Simulator -- Set properties and screen sizes here - will run once when the script is loaded
    simulator = simulator
    simulator:setScreen(1, "3x3")
    simulator:setProperty("Resolution", 25)
    simulator:setProperty("Min Distance", 25)
    simulator:setProperty("Max Distance", 1000)
    simulator:setProperty("Sweep Speed", 1)
    simulator:setProperty("Max gScore", 150)
    simulator:setProperty("Smoothing Factor", 0.1)

    -- Runs every tick just before onTick; allows you to simulate the inputs changing
    ---@param simulator Simulator Use simulator:<function>() to set inputs etc.
    ---@param ticks     number Number of ticks since simulator started
    function onLBSimulatorTick(simulator, ticks)

        -- touchscreen defaults
        local screenConnection = simulator:getTouchScreen(1)
        --simulator:setInputBool(1, screenConnection.isTouched)
        --simulator:setInputNumber(1, screenConnection.width)
        --simulator:setInputNumber(2, screenConnection.height)
        --simulator:setInputNumber(3, screenConnection.touchX)
        --simulator:setInputNumber(4, screenConnection.touchY)

        -- NEW! button/slider options from the UI
        --simulator:setInputBool(31, simulator:getIsClicked(1))       -- if button 1 is clicked, provide an ON pulse for input.getBool(31)
        --simulator:setInputNumber(31, simulator:getSlider(1))        -- set input 31 to the value of slider 1

        --simulator:setInputBool(32, simulator:getIsToggled(2))       -- make button 2 a toggle, for input.getBool(32)
        --simulator:setInputNumber(32, simulator:getSlider(2) * 50)   -- set input 32 to the value from slider 2 * 50

        simulator:setInputBool(1, simulator:getIsToggled(1))       -- make button 1 a toggle, for input.getBool(32)

        simulator:setInputNumber(18, 999)  -- set distance reading for laser

        simulator:setInputNumber(19, -100000)  -- set target x coord
        simulator:setInputNumber(20, -100000)  -- set target y coord


        if simulator:getIsToggled(1) then --if button 1 is toggled
           simulator:setInputNumber(3, yPosition)   -- set y coord
           simulator:setInputNumber(1, xPosition)   -- set x coord
           yPosition = yPosition + math.cos(targetYaw)
           xPosition = xPosition + math.sin(targetYaw)
        end
        if simulator:getIsToggled(2) then --if button 2 is toggled
            simulator:setInputNumber(19, 100000)  -- set target x coord
            simulator:setInputNumber(20, -50000)  -- set target y coord
        end


        --convert screen touch to map coordinate
        if screenConnection.isTouched then
            local node = {}
            node.x, node.y = map.screenToMap(currentGPSPositionX, currentGPSPositionY, scale, screenConnection.width, screenConnection.height, screenConnection.touchX, screenConnection.touchY)
            addObstructedNode(node)
        end
    end;
end
---@endsection


--[====[ IN-GAME CODE ]====]

-- try require("Folder.Filename") to include code from another file in this, so you can store code in libraries
-- the "LifeBoatAPI" is included by default in /_build/libs/ - you can use require("LifeBoatAPI") to get this, and use all the LifeBoatAPI.<functions>!

ticks = 0

buffersize = 5 --size of the buffer for the yaw values, this buffer compensates for 5 ticks of delay between yaw values and sensor readings

resolution = property.getNumber("Resolution") --how many nodes the map is divided into
maxLaserRange = property.getNumber("Max Distance") --maximum distance the laser can detect
minLaserRange = property.getNumber("Min Distance") --minimum distance the laser can detect
laserSweepSpeed = property.getNumber("Sweep Speed") --speed of the laser sweep
maxGScore = property.getNumber("Max gScore") --maximum value for the gScore in the A* algorithm
smoothingFactor = property.getNumber("Smoothing Factor") --smoothing factor for the path

scale = maxLaserRange/500 --scale of the map on the screen, based on the max distance of laser detection
cellSize = (maxLaserRange*2)/resolution --size of each square node on the map
epsilon = cellSize/2 --distance between centre of node and edge of node

sweepRight = true --global boolean to determine the direction of the laser sweep
laserSweepAngle = 0 --global integer to store the current angle of the laser sweep

obstructedNodesTable = {} --define the table to store detected obstructions

function buffer(value, bufferTable) -- this buffer is necessary to allow the data from sensors to catch up with the yaw value generated by the code
    table.insert(bufferTable,1,value) --values are added to the buffer
    table.remove(bufferTable, buffersize + 1) -- the oldest value is removed from the buffer
    return(bufferTable[buffersize]) -- the value at the end of the buffer is returned
end

function intialiseBuffer() --this function initialises the buffer table with values of 0
    local bufferTable = {}
    for i =1, buffersize do
        bufferTable[i] = 0
    end
    return bufferTable
end

yawbuffer = intialiseBuffer() --initialise the buffer table for the yaw values


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
    --we then rotate this vector in the x and y direction based on the pitch and roll of the vehicle, as well as the desired sweep angle
    -- x and y axis rotation
    X2 = (math.sin(laserSweepAngleInRadians))
    Y2 = - ((math.cos(laserSweepAngleInRadians)) * math.sin(currentPitch))
    Z2 =  ((math.cos(laserSweepAngleInRadians)) * math.cos(currentPitch))

    -- i honestly cant remember exactly what all this math does, but it works so im not going to touch it

    --perform Z axis rotation and calculate new rotation


    currentRoll = math.atan((math.sin(currentRoll)) ,(math.cos(currentRoll)))


    --Z axis rotation
    X5 = (X2*math.cos(currentRoll)) - (Y2 * math.sin(currentRoll))
    Y5 = (X2 * math.sin(currentRoll)) + (Y2 * math.cos(currentRoll))
    

    laserYaw = math.atan(X5,Z2) -- calculate the yaw value for the laser

    output.setNumber(1,8 * laserYaw/(2*math.pi)) -- output the pitch value for the laser
    
    -- as the way the laser angle is controlled works a bit funny, we need to apply another rotation to make it behave as expected

    --y axis rotation
    Z4 = (Z2 * math.cos(-laserYaw)) - (X5 * math.sin(-laserYaw))

    output.setNumber(2,8 * math.atan(Y5,Z4)/(2*math.pi)) -- output the yaw value for the laser

end

function getLaserTargetCoordinates() --gets the coords of the laser point

    delayedYaw = buffer(laserYaw, yawbuffer) --get the delayed yaw value

    local laserTargetX = currentGPSPositionX + math.sin(delayedYaw - currentAzimuth) * laserDistanceReading --calculate the x coordinate of the laser point
    local laserTargetY = currentGPSPositionY + math.cos(delayedYaw - currentAzimuth) * laserDistanceReading --calculate the y coordinate of the laser point

    return {x = laserTargetX, y = laserTargetY} --return the coordinates of the laser point

end

function addObstructedNode(worldCoord)-- add a node to the obstruction graph
    if laserDistanceReading < maxLaserRange and laserDistanceReading > minLaserRange then --if the detected target is within the maximum specified range
        local tableCoord = worldToTable(worldCoord) --convert the world coordinates to a suitable table entry

        if not isObstructed(tableCoord) then --if the node is not already in the graph
            table.insert(obstructedNodesTable,1,tableCoord) --add the node to the graph
        end
    end
end

function worldToTable(worldCoord) --converts world coordinates to table coordinates
    return {x = math.floor(worldCoord.x/cellSize), y = math.floor(worldCoord.y/cellSize)}
end

function tableToWorld(tableCoord) -- converts table coordinates to world coordinates
    return {x = (tableCoord.x + 0.5) * cellSize, y = (tableCoord.y + 0.5) * cellSize}
end

function isObstructed(node) --check if a node is obstructed
    for _, tableNode in ipairs(obstructedNodesTable) do --iterate through the table of obstructions
        if tableNode.x == node.x and tableNode.y == node.y then --if the node is in the table, return true
            return true
        end
    end
    return false
end

-- euclidian distance using pythagoras' theorem
function euclidian(node, goal)
    return math.sqrt((node.x - goal.x)^2 + (node.y - goal.y)^2)
end

-- octile distance heuristic for A* algorithm
function heuristic(node, goal)
    dx = math.abs(node.x - goal.x) --calculate the difference in x coordinates
    dy = math.abs(node.y - goal.y) --calculate the difference in y coordinates

    return (dx + dy) - 0.6 * math.min(dx, dy) --return the octile distance 
end

-- Function to retrieve neighbors of a node
function getNeighbors(node)
    local neighbors = {
        {x = node.x + 1, y = node.y},
        {x = node.x - 1, y = node.y},
        {x = node.x, y = node.y + 1},
        {x = node.x, y = node.y - 1}
    }

    --if neither of the diagonals are obstructed, add them to the neighbors list
    if not (isObstructed(neighbors[1]) or isObstructed(neighbors[3])) then
        table.insert(neighbors, {x = node.x + 1, y = node.y + 1})
    end
    if not (isObstructed(neighbors[1]) or isObstructed(neighbors[4])) then
        table.insert(neighbors, {x = node.x + 1, y = node.y - 1})
    end
    if not (isObstructed(neighbors[2]) or isObstructed(neighbors[3])) then
        table.insert(neighbors, {x = node.x - 1, y = node.y + 1})
    end
    if not (isObstructed(neighbors[2]) or isObstructed(neighbors[4])) then
        table.insert(neighbors, {x = node.x - 1, y = node.y - 1})
    end
    return neighbors
end

function reconstructPath(cameFrom, current) --reconstruct the path from the cameFrom map
    local totalPath = {current}
    while cameFrom[current.x .. "," .. current.y] do
        current = cameFrom[current.x .. "," .. current.y]
        table.insert(totalPath, 1, current)
    end
    return totalPath
end

function checkPathForObstructions(path) --check if the path is obstructed
    if path == nil then
        return true
    end
    for _, node in ipairs(path) do
        if isObstructed(node) then
            return true
        end
    end
    return false
end

openSet = {} -- The set of nodes to be evaluated
cameFrom = {} -- The map of navigated nodes
gScore = {} -- Cost from start along best known path
fScore = {} -- Estimated total cost from start to goal

start = {x=0, y=0} -- Start node
current = {x=0, y=0} -- Current node

path = {}

refreshPathFinding = true -- Flag to refresh the path
pathFound = false -- Flag to check if a path has been found

maxGScoreReached = false

-- A* pathfinding algorithm
function aStar(goal)
    -- check if start or goal have changed
    if refreshPathFinding then
        refreshPathFinding = false

        start = currentNode --set the start node for the pathfinding algorithm

        openSet = {start} -- The set of nodes to be evaluated
        cameFrom = {} -- The map of navigated nodes
        current = start -- Current node

        gScore = {[start.x .. "," .. start.y] = 0} -- Cost from start along best known path
        fScore = {[start.x .. "," .. start.y] = heuristic(start, goal)} -- Estimated total cost from start to goal

        maxGScoreReached = false
    end

    pathFound = (current.x == goal.x and current.y == goal.y or maxGScoreReached) --check if the goal has been reached, or the maximum gScore has been reached

    if #openSet > 0 and not (pathFound) then
        -- Find the node in openSet with the lowest fScore
        table.sort(openSet, function(a, b) return fScore[a.x .. "," .. a.y] < fScore[b.x .. "," .. b.y] end)
        current = table.remove(openSet, 1)        

        -- get all neighbors of the current node
        for index, neighbor in ipairs(getNeighbors(current)) do
            if not isObstructed(neighbor) then -- check if the neighbor node is obstructed
                local tentative_gScore
                if index > 4 then -- if the neighbor is a diagonal node, increment the gScore by 1.4
                    tentative_gScore = gScore[current.x .. "," .. current.y] + 1.3
                else -- if the neighbor is not a diagonal node, increment the gScore by 1
                    tentative_gScore = gScore[current.x .. "," .. current.y] + 1
                end
                local neighborKey = neighbor.x .. "," .. neighbor.y
                if tentative_gScore < (gScore[neighborKey] or math.huge) then
                    cameFrom[neighborKey] = current -- best path so far
                    gScore[neighborKey] = tentative_gScore -- update the gScore
                    fScore[neighborKey] = tentative_gScore + heuristic(neighbor, goal) -- update the fScore
                    
                    if gScore[neighborKey] > maxGScore then --if the gScore is greater than the maximum gScore, stop the search
                        maxGScoreReached = true
                    else
                        table.insert(openSet, neighbor) -- add the neighbor to the openSet for evaluation
                    end
                end
            end
        end
        path = reconstructPath(cameFrom, current) --construct the most efficient path
        table.insert(path, #path + 1, goal) --add the start node to the path
    end    
    if contains(path, currentNode) then
        while contains(path, currentNode) do
            table.remove(path, 1)
        end
        table.insert(path, 1, currentNode)
    end

    return path --merge the path and return it
end

function contains(table, element) --check if a table contains an element
    for _, value in ipairs(table) do
        if value.x == element.x and value.y == element.y then
            return true
        end
    end
    return false
end

targetYaw = 0 --set the target yaw to 0

function getTargetYaw(target) --get the real world azimuth of the target relative to the current position
    return math.atan(target.x-currentGPSPositionX,target.y-currentGPSPositionY) --calculate the azimuth 
end

function navigate(path) --navigate to the target
    if path[2] then --if there is a next node in the path
        targetYaw = getTargetYaw(calculateTargetPoint(path)) --get the azimuth of the next node in the path
    else
        targetYaw = getTargetYaw(path[1]) --get the azimuth of the last node in the path
    end
    yawOutput = - (math.fmod(((-currentAzimuth/ (2*math.pi)))-(targetYaw/(2*math.pi))+1.5,1)-0.5) --calculate the difference between current azimuth and target azimuth in turns
end

function calculateTargetPoint(path) --calculate the optimal target in the path
    targetNode = tableToWorld(path[2])
    previousTargetNode = tableToWorld(path[1])

    totalDistance = euclidian(previousTargetNode, targetNode)
    remainingDistance = euclidian({x = currentGPSPositionX, y = currentGPSPositionY}, targetNode)

    -- interpolate the target point based on the distance to the next node
    targetPoint = {x = targetNode.x - (targetNode.x - previousTargetNode.x) * (remainingDistance/totalDistance * (1 - smoothingFactor)), y = targetNode.y - (targetNode.y - previousTargetNode.y) * (remainingDistance/totalDistance * (1 - smoothingFactor))}
    
  
    return targetPoint
end

function mergePathNodes(path) --remove redundant nodes from the path
    local mergedPath = {}
    local previousNode = path[1]
    local previousVector = {x = 0, y = 0}
    for _, node in ipairs(path) do
        local vector = toVector(previousNode.x, previousNode.y, node.x, node.y) --calculate the vector between the previous node and the current node
        if vector.x ~= previousVector.x or vector.y ~= previousVector.y then --if the vector is not the same as the previous vector, add the node to the merged path
            table.insert(mergedPath, previousNode)
            previousVector = vector --set the previous vector to the current vector
        end
        previousNode = node --set the previous node to the current node
    end
    table.insert(mergedPath, previousNode) --add the last node to the merged path
    return mergedPath --return the merged path

end

function toVector(x1, y1, x2, y2) --convert two points to a vector
    return {x = x2 - x1, y = y2 - y1}
end

oldGoal = {x = 0, y = 0} --set the old goal to 0,0

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

    currentNode = worldToTable({x = currentGPSPositionX, y = currentGPSPositionY}) --convert the current GPS position to a table entry
    goalNode = worldToTable({x = gpsTargetX, y = gpsTargetY}) --convert the target GPS position to a table entry

    output.setNumber(3,cellSize)

    --when input 1 is on, go through the calculations
    if scriptIsEnabled then
        laserSweep() --run the laser sweep function
        addObstructedNode(getLaserTargetCoordinates()) --run the addObstructedNode function

        if start.x == 0 or start.y == 0 then 
            start = currentNode --set the start node for the pathfinding algorithm
        end

        path = aStar(goalNode) --run the pathfinding algorithm

        if checkPathForObstructions(path) then --if the path is obstructed, restart pathfinding
            start = currentNode
            refreshPathFinding = true
        end
        if goalNode.x ~= oldGoal.x or goalNode.y ~= oldGoal.y then --if the goal has changed, restart pathfinding
            refreshPathFinding = true
            oldGoal = goalNode
        end
        

        navigate(mergePathNodes(path)) --get the yaw output for the next node in the path
        output.setNumber(4, 4*yawOutput)

    end


end

function onDraw()
    sWidth = screen.getWidth()
    sHeight = screen.getHeight()
    screen.drawMap(currentGPSPositionX,currentGPSPositionY, scale)-- draw the map to the screen

    --screen.setColor(0,0,255) -- plots the final yaw
    --screen.drawLine(sWidth/2, sHeight/2,(math.sin(finalYaw)*sWidth/2) + sWidth/2, sHeight/2-(math.cos(finalYaw)* sHeight/2))

    screen.setColor(0,255,0) -- plots the target azimuth
    screen.drawLine(sWidth/2, sHeight/2,(math.sin(targetYaw)*sWidth/2) + sWidth/2, sHeight/2-(math.cos(targetYaw)* sHeight/2))

    screen.setColor(255,0,0) -- plots the current azimuth
    screen.drawLine(sWidth/2, sHeight/2,(math.sin(-currentAzimuth)*sWidth/2) + sWidth/2, sHeight/2-(math.cos(currentAzimuth)* sHeight/2))
    
    screen.setColor(255,255,255) -- plots the laser azimuth
    screen.drawLine(sWidth/2, sHeight/2 , (math.sin(laserSweepAngle * 2 * math.pi -currentAzimuth)*sWidth/2) + sWidth/2, sHeight/2-(math.cos(laserSweepAngle * 2 * math.pi -currentAzimuth)* sHeight/2))

    for _, node in ipairs(obstructedNodesTable) do --for each node in the obstruction table, plot a white square on the map
        screenX, screenY = map.mapToScreen(currentGPSPositionX, currentGPSPositionY, scale, sWidth, sHeight, node.x * cellSize, (node.y + 1) * cellSize)
        rectangleWidth = (sWidth/resolution)
        rectangleHeight = (sHeight/resolution)
        screen.drawRect(screenX,screenY,rectangleWidth,rectangleHeight)
    end

    if path then --if a path has been found
        previousNode = currentNode -- set the previous node to the start node
        previousNodeScreenCoord = {x = 0, y = 0}
        currentNodeScreenCoord = {x = 0, y = 0}

        screen.setColor(255,0,0)--set the colour of the path to red
        if pathFound then --if the path has been found
            screen.setColor(0,255,0)--set the colour of the path to green
        end

        for _, node in ipairs(path) do --for each node in the path, plot a green line on the map
            previousNodeScreenCoord.x, previousNodeScreenCoord.y = map.mapToScreen(currentGPSPositionX, currentGPSPositionY, scale, sWidth, sHeight, previousNode.x * cellSize + epsilon, previousNode.y * cellSize + epsilon)
            currentNodeScreenCoord.x, currentNodeScreenCoord.y = map.mapToScreen(currentGPSPositionX, currentGPSPositionY, scale, sWidth, sHeight, node.x * cellSize + epsilon, node.y * cellSize + epsilon)
            screen.drawLine(previousNodeScreenCoord.x, previousNodeScreenCoord.y, currentNodeScreenCoord.x, currentNodeScreenCoord.y)
        
            previousNode = node --copy the current node for the next iteration
        end
    end

    
end



