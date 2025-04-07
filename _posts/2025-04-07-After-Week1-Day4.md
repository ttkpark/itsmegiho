---
layout: post
title: "After-Week1_Day4"
date : 2025-04-07
categories: [SFU_project]
---
Janus는
Core + Plugins 구조로 이루어져 있다.

Transports(전송법)
안드로이드에서도  WebSocket 시그널링을 활용한다니.

서버 로그 분석
- 어디에 있는 서버 로그?
- ICE Candidate 교환 -> DRLS Handshake 후 SRTP로 전송


(아직 구현 X) 시그널링 과정:
create (session 생성)
attach (plugin 연결)
offer (SDP)
trickle (ICE candidate)
answer (Janus → 클라이언트)
다음 주(혹은 Day 5)에서 본격 적용


4. 숙제(퀴즈)
1. Server Log 퀴즈
- ICE Gathering, DTLS handshake, SRTP 적용 순서를 간단히 적어보세요.
ICE -> DTLS -> SRTP
- 로그에서 “Plugin handle created”는 어떤 의미일까요?
Echo test의 plugin이 실행되기 위한 핸들이 만들어졌다??
2. Custom Config
- debug_level을 높이면 어떤 이점과 단점이 생기는지 두 가지씩 적어보세요.
자세한 정보를 알 수 있다, 로그를 놓칠 일이 없다 / 선별이 어렵고, 용량이 너무 크다
- VideoRoom plugin에서 max_publishers를 3으로 설정했을 때, 4번째 참가자가 publish하면 어떻게 될까요?
안해봤지만, 하면 연결 오류가 날 것 같다.
publish라는 용어를 쓰는지 알았다.  앞으로 publisher라고 업스트림을 정의하면 되겠다.
3. 시그널링 JSON
- attach 요청과 join 요청의 차이를 짧게 설명해보세요.
?
- transaction 필드는 왜 필요한가요?
?