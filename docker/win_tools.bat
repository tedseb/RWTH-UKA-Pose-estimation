echo off
set arg1=%1
IF "%arg1%" == "pull" (
    GOTO pull
)
IF "%arg1%" == "setup" (
    GOTO setup
) 
IF "%arg1%" == "start" (
    GOTO getIpAndStart
) 
IF "%arg1%" == "into" (
    GOTO into
) 
IF "%arg1%" == "test" (
    GOTO test
) 


Echo ---------------- Help ----------------
Echo pull         Pull the current version.
Echo setup        Setup a new repo. 
Echo start        Start docker.
ECHO into         Go into the docker. 
GOTO :EOF 

:pull
   Echo pull
   docker login registry.git.rwth-aachen.de
   docker pull registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update
   GOTO :EOF 

:setup
   Echo setup
   git config core.symlinks true
   git config core.autocrlf input
   git read-tree --empty
   git reset --hard HEAD
   GOTO :EOF 

:getIpAndStart
    REM set str = %ipconfig | findstr /i "ipv4"
    ipconfig | findstr /i "ipv4" > ip.txt
    REM "   IPv4-Adresse  . . . . . . . . . . : 192.168.178.21"
    for /f "tokens=13" %%G IN (ip.txt) DO (
        set ip=%%G
        del ip.txt
        GOTO start
    ) 

:start
   Echo Run Docker
   ECHO The used IP: %ip%
   REM docker run -it --rm --name TrainerAi -v ${pwd}:/trainerai -v /trainerai/node_modules registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update
   SET CURRENTDIR=%~dp0..
   ECHO Mount Docker in %CURRENTDIR%
   docker run -it --rm --name TrainerAi -e DISPLAY=%ip%:0.0 -v %CURRENTDIR%:/trainerai -v /trainerai/node_modules registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update
   GOTO :EOF 

:into
   Echo into
   docker exec -it TrainerAi /bin/bash
   GOTO :EOF 