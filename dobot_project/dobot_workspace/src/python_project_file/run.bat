@echo off
chcp 65001 > nul
title Enhanced Dobot Robot System

echo.
echo 🤖========================================================🤖
echo      Enhanced Dobot Robot & YOLO Object Detection System
echo 🤖========================================================🤖
echo.

REM Python 설치 확인
python --version > nul 2>&1
if errorlevel 1 (
    echo ❌ Python이 설치되지 않았거나 PATH에 없습니다.
    echo 💡 Python 3.8 이상을 설치하고 PATH에 추가하세요.
    echo    https://www.python.org/downloads/
    pause
    exit /b 1
)

echo ✅ Python 확인됨
python --version

REM 필수 파일 확인
if not exist "main.py" (
    echo ❌ main.py 파일을 찾을 수 없습니다.
    echo 💡 모든 파일이 같은 폴더에 있는지 확인하세요.
    pause
    exit /b 1
)

if not exist "dobot_api_handler.py" (
    echo ❌ dobot_api_handler.py 파일을 찾을 수 없습니다.
    echo 💡 모든 파일이 같은 폴더에 있는지 확인하세요.
    pause
    exit /b 1
)

echo ✅ 필수 파일 확인됨

REM 기본 패키지 확인
echo.
echo 📦 필수 패키지 확인 중...
python -c "import numpy, matplotlib, cv2, PIL" > nul 2>&1
if errorlevel 1 (
    echo ⚠️ 일부 필수 패키지가 없습니다.
    echo 💡 자동 설치를 시도하시겠습니까? (Y/N)
    set /p install_choice=입력: 
    if /i "%install_choice%"=="Y" (
        echo 🔄 패키지 설치 중...
        pip install numpy matplotlib opencv-python pillow ultralytics pyyaml
        if errorlevel 1 (
            echo ❌ 패키지 설치 실패
            echo 💡 수동으로 설치하세요: pip install -r requirements.txt
            pause
            exit /b 1
        )
        echo ✅ 패키지 설치 완료
    ) else (
        echo ⚠️ 패키지 없이 실행 시도 중...
    )
) else (
    echo ✅ 필수 패키지 확인됨
)

REM Dobot API 확인
echo.
echo 🤖 Dobot API 확인 중...
python -c "from dobot_api_handler import DOBOT_API_AVAILABLE; print('API Available:', DOBOT_API_AVAILABLE)" 2>nul
if errorlevel 1 (
    echo ⚠️ Dobot API 핸들러 로드 실패
    echo 💡 diagnose_dobot.py를 실행하여 문제를 확인하세요.
) else (
    echo ✅ Dobot API 핸들러 로드 성공
)

REM 로그 디렉터리 생성
if not exist "logs" mkdir logs
echo ✅ 로그 디렉터리 준비됨

echo.
echo 🚀 시스템 시작 중...
echo ======================================================
echo 💡 팁: 
echo    - Dobot API가 없어도 시뮬레이션 모드로 실행됩니다
echo    - 실제 로봇 연결을 위해서는 'pip install pydobot' 실행
echo    - 문제 발생 시 diagnose_dobot.py 실행하여 진단
echo ======================================================
echo.

REM 메인 프로그램 실행
python main.py

REM 종료 처리
echo.
if errorlevel 1 (
    echo ❌ 프로그램이 오류와 함께 종료되었습니다.
    echo 💡 logs/robot_system.log 파일을 확인하세요.
) else (
    echo ✅ 프로그램이 정상적으로 종료되었습니다.
)

echo.
echo 🔧 문제 해결 도구:
echo    - python diagnose_dobot.py  (Dobot API 진단)
echo    - python test_system.py     (시스템 테스트)
echo    - python setup.py           (재설치)
echo.

pause
