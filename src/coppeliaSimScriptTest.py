function sysCall_init()
    -- IK (simple way)
    local simBase=sim.getObjectHandle('UR5')
    local simTip=sim.getObjectHandle('UR5tip')
    local simTarget=sim.getObjectHandle('UR5target')
    -- create an IK environment:
    ikEnv=simIK.createEnvironment()
    -- create an IK group: 
    ikGroup_undamped=simIK.createIkGroup(ikEnv)
    -- set its resolution method to undamped: 
    simIK.setIkGroupCalculation(ikEnv,ikGroup_undamped,simIK.method_pseudo_inverse,0,6)
    -- create an IK element based on the scene content: 
    simIK.addIkElementFromScene(ikEnv,ikGroup_undamped,simBase,simTip,simTarget,simIK.constraint_pose)
    -- create another IK group: 
    ikGroup_damped=simIK.createIkGroup(ikEnv)
    -- set its resolution method to damped: 
    simIK.setIkGroupCalculation(ikEnv,ikGroup_damped,simIK.method_damped_least_squares,1,99)
    -- create an IK element based on the scene content: 
    simIK.addIkElementFromScene(ikEnv,ikGroup_damped,simBase,simTip,simTarget,simIK.constraint_pose) 
    -- IK (simple way)
    
    --Path Follow
    objectToFollowPath=sim.getObjectHandle('UR5target')
    path=sim.getObjectHandle('Path')
    pathData=sim.unpackDoubleTable(sim.readCustomDataBlock(path,'PATH'))
    local m=Matrix(#pathData//7,7,pathData)
    pathPositions=m:slice(1,1,m:rows(),3):data()
    pathQuaternions=m:slice(1,4,m:rows(),7):data()
    pathLengths,totalLength=sim.getPathLengths(pathPositions,3)
    velocity=0.04 -- m/s
    posAlongPath=0
    previousSimulationTime=0
    corout=coroutine.create(coroutineMain)
    --Path Follow
    
    -- Joint Handle
    JH = {}
    JH[1] = sim.getObjectHandle('UR5_joint1')
    JH[2] = sim.getObjectHandle('UR5_joint2')
    JH[3] = sim.getObjectHandle('UR5_joint3')
    JH[4] = sim.getObjectHandle('UR5_joint4')
    JH[5] = sim.getObjectHandle('UR5_joint5')
    JH[6] = sim.getObjectHandle('UR5_joint6')
    -- Joint Handle
end

function sysCall_actuation()
    -- IK (simple way)
    -- try to solve with the undamped method:
    if simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_undamped,true)==simIK.result_fail then 
        -- the position/orientation could not be reached.
        -- try to solve with the damped method:
        simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_damped)
        -- We display a IK failure report message: 
        if not ikFailedReportHandle then 
            ikFailedReportHandle=sim.displayDialog("IK failure report","IK solver failed.",
                sim.dlgstyle_message,false,"",nil,{1,0.7,0,0,0,0})
        end
    else
        if ikFailedReportHandle then
        	-- We close any report message about IK failure:
            sim.endDialog(ikFailedReportHandle) 
            ikFailedReportHandle=nil
        end
    end
    -- IK (simple way)
    
    --Path Follow
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
    --Path Follow
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- erase the IK environment: 
    simIK.eraseEnvironment(ikEnv) 
end

-- See the user manual or the available code snippets for additional callback functions and details
function coroutineMain()
    sim.setThreadAutomaticSwitch(false)
    while true do
        local t=sim.getSimulationTime()
        posAlongPath=posAlongPath+velocity*(t-previousSimulationTime)
        posAlongPath=posAlongPath % totalLength
        local pos=sim.getPathInterpolatedConfig(pathPositions,pathLengths,posAlongPath)
        local quat=sim.getPathInterpolatedConfig(pathQuaternions,pathLengths,
                                                 posAlongPath,nil,{2,2,2,2})
        sim.setObjectPosition(objectToFollowPath,path,pos)
        sim.setObjectQuaternion(objectToFollowPath,path,quat)
        previousSimulationTime=t
        sim.switchThread()
    end
end