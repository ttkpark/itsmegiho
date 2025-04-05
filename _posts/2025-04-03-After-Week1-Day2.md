---
layout: post
title: "After-Week1-Day2"
date : 2025-04-03
categories: [SFU_project]
---
# SFU 심화
1. Selective Forwarding
  디코딩 없이 그대로 포워딩 / RTP Session
  내가 기존에 설계했던 음성전송방식은 RTP Session인가? / webrtc 앱은 ICE가 교환되면 RTP 방식으로 데이터를 주고받는가?
  믹싱하는 MCU 방식보다 성능이 덜하다 (?)
2. 오픈소스 SFU 비교
  1) Jauns (C, Plugin 구조) : 성숙한 커뮤니티 / C 빌드가 어려울 수 있다
  2) Mediasoup (Node.js + C++) : 고성능 / c++빌드가 어려울 수 있다
  3) Pion (Go) : 쉽고 간단히 시작 가능 / 플러그인이 풍부하지 않을 수 있다

3. Jauns 환경
  STUN은 구글 STUN 서버 활용
  `coturn`과 같은 TURN서버를 둘 수도 있다.
  
  site를 보고 공부한 내용 :
  ```
  기본은 아무것도 없고, 다양한 기능(통신, JSON메시지 교환, 웹브라우저에서 보기) 모두는 plugin으로 공급된다.
  심지어 로깅도 stdout로 파일로 꺼낼 수도 있지만, 로깅 플러그인(Logger API)도 존재한다.
  ```
  - Janus HTTP (TCP 8088), WS (TCP 8188)
  - UDP range 10000-60000 (if needed for WebRTC)
  
  - 주로 “Janus”를 사용하되, Windows 환경에서 WSL이나 Docker로 빌드 가능
  - 방화벽과 TURN 설정은 꼭 챙겨야 함 (다자 연결, NAT 환경)


4. `git branch` 배우기
  ```bash
  git checkout main
  git pull
  git checkout -b week1-day2
  
  mkdir docs
  cd docs

  # SFU-comparison.md 파일 생성, 내용 입력

  git add .
  git commit -m "Add SFU-copmparison.md for Day 2" 

  git checkout main
  git pull
  git merge week1-day2
  git push
  ```
# 퀴즈
1. SFU 비교 퀴즈
- Janus와 Mediasoup 중, 본인이 선호할 것 같은 이유 2가지를 적어보세요. (언어/커뮤니티/성능 등)
  난 Janus를 선호한다. C언어 기반이며 더 능숙하고, 커뮤니티도 풍부하며, 라즈베리 파이에 들어갈 만한 설정도 가능하기 때문이다.
2. Windows 서버 배포
- 방화벽에서 UDP 다중 포트를 열어야 하는 이유는 무엇인가?
  다양한 클라이언트의 스트림을 받기 위해서는 UDP 포트로 RTP Session이 열리기 때문이다.
3. TURN 서버
- TURN이 필요한 상황 1가지를 예시 들고, 없으면 어떻게 되는지 설명해보세요. (예: Symmetric NAT)
  NAT 종류가 여러가지가 있는데, 그 중 나가는 포트와 들어오는 포트가 서로 대칭적으로 정해지는 Symmetric NAT는 UDP Hole punching라는 NAT 넘어서 P2P 연결이 가능한 형태가 되지만, 비대칭적인 몇몇 NAT 설정에는 NAT넘어서 P2P 송수신이 불가능하다. 그럴 때는 외부 서버와 연결하여 데이터를 주고받아야 한다. Janus는 외부 서버가 될텐데 왜 TURN 서버가 필요하고 존재하는지 모르겠다.