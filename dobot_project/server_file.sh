#!/bin/bash

cd "$(dirname "$0")"

echo "🤫 조용한 모드로 Python 서버 실행"
echo "HTTP 요청 로그를 표시하지 않습니다."
echo ""

# Python 서버 디렉토리로 이동
cd python_server

# 옵션 선택

echo "🔊 일반 모드로 실행"
python3 main.py
;;
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
exec(open('main.py').read())
"
        ;;
esac
