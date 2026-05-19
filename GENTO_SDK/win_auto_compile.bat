@echo off
chcp 65001 >nul

echo complie dll...
del /F /Q C_SDK\libMarvinSDK.dll 2>nul
cd C_SDK &&g++ ./L0Control/*.cpp ./FileClient/*.cpp ./L1Robot/*.cpp ./Kinematics/*.cpp ./Kinematics/ArmKinematics/*.cpp ./Kinematics/BaseMath/*.cpp ./Kinematics/DynaIdent/*.cpp ./Kinematics/KineCommon/*.cpp ./Kinematics/MotionPlanner/*.cpp ./Kinematics/SkyeBodyKinematics/*.cpp -Wall -O2 -shared -I./Common -I./Kinematics -I./Kinematics/ArmKinematics -I./Kinematics/BaseMath -I./Kinematics/DynaIdent -I./Kinematics/KineCommon -I./Kinematics/MotionPlanner -I./Kinematics/SkyeBodyKinematics -I./FileClient -I./L0Control -I./L1Robot -o libGentoSDKPY.dll -DL1_SDK_EXPORTS -DCMPL_WIN -static -static-libgcc -static-libstdc++ -lws2_32 -lwinmm
echo ✓ Compilation completed
cd ..
copy /Y C_SDK\libGentoSDKPY.dll PYTHON_SDK\ >nul
echo ✓ Copy the dll to  ./PYTHON_SDK/

pause