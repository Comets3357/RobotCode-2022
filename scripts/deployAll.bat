@echo off
rem Execute gradle build
cd ..
echo Building robot code
call gradlew build
rem Execute gradle dashboard
echo Launching dashboard
call gradlew dashboard