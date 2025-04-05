---
layout: post
title: "After-Week1-Day1"
date : 2025-04-02
categories: [SFU_project]
---
# git에 대해 배우기
- `git init`
  현재 폴더에 git을 초기화함.
- `git remote add origin https://~`
  원격 Repository 연결
- `git remote -v`
  확인
  origin  https://github.com/username/sfu-project.git (fetch)
  origin  https://github.com/username/sfu-project.git (push)
- `git add .`
- `git commit -m "commit ment"`
- `git push`

# 퀴즈
- Git 개념 퀴즈
  - `git clone`과 `git pull`의 차이??
    clone는 원격 저장소에서 내용을 그대로 가져오는 것이고, pull은
  - `git add .`의 역할
    디렉터리의 변경사항을 추적하고, 그 사항을 반영할 준비를 하는 명령어.
- WebRTC 기본 퀴즈
  - P2P 연결에서 SDP가 어떤 정보를 담는지 2가지 이상 쓰세요.
    세션 정보, 전달하려는 내용용
  - ICE Candidate는 왜 필요한가요?
    시그널링 과정에서 데이터를 인터넷상에 전달할 원격 주소 등 필수 정보를 알아야만 P2P 연결을 할 수 있기 때문
- SFU/MCU/Mesh 단답형
  - SFU 구조가 Mesh 대비 가지는 장점 2가지는?
    6인 이상의 대화 상황에서 적용할 수 있다.
    대역폭의 사용 및 단말기의 처리량이 대폭 감소한다.