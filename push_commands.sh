#!/bin/bash
# GitHub repository push commands
# Repository를 생성한 후 아래 명령어를 실행하세요

# GitHub username을 입력하세요
GITHUB_USERNAME="kuks2309"

# 원격 저장소 추가
git remote add origin https://github.com/${GITHUB_USERNAME}/Charging_Robot_Operation.git

# 기본 브랜치를 main으로 변경 (선택사항)
git branch -M main

# Push to GitHub
git push -u origin main

echo "✅ Successfully pushed to GitHub!"
