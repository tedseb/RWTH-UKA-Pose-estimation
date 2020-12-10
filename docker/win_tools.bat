echo off
set arg1=%1
IF "%arg1%" == "pull" (
    GOTO pull
)
IF "%arg1%" == "setup" (
    GOTO setup
) 
IF "%arg1%" == "start" (
    GOTO start
) 
IF "%arg1%" == "into" (
    GOTO into
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
   git reset --hard HEAD
   GOTO :EOF 

:start
   Echo Run Docker
   REM docker run -it --rm --name TrainerAi -v ${pwd}:/trainerai -v /trainerai/node_modules registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update
   SET CURRENTDIR=%~dp0..
   ECHO Mount Docker in %CURRENTDIR%
   docker run -it --rm --name TrainerAi -v %CURRENTDIR%:/trainerai -v /trainerai/node_modules registry.git.rwth-aachen.de/trainerai/core/trainerai-dev-update
   GOTO :EOF 

:into
   Echo into
   docker exec -it TrainerAi /bin/bash
   GOTO :EOF 