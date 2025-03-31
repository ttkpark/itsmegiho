아래는 **Week 1 (Day 1 ~ Day 5)의 상세 강의안**을 마크다운 형식으로 정리한 내용입니다. 각 Day별로 **목표**, **이론**, **실습**, **숙제(퀴즈)**, 그리고 마지막에 **코멘트/팁** 섹션을 추가했습니다. 이 강의안은 이전에 합의한 “SFU 프로젝트의 전체 주차별 개요(Week 1 ~ Week 12)”를 바탕으로, **Week 1: 기초 다지기 & SFU 개념 잡기**라는 목표에 초점을 맞추고 있습니다.

---

# **Week 1: 기초 다지기 & SFU 개념**

## **Day 1**

### **목표**
- Git/GitHub 기초 사용법 습득 및 실습  
- WebRTC(특히 P2P) 개념 간단 복습  
- SFU/MCU/Mesh의 차이를 이론적으로 파악하기 위한 준비

### **이론**
1. **Git/GitHub 기본 개념**  
   - 버전 관리의 필요성  
   - 로컬 저장소, 원격 저장소, 리포지토리 개념  
   - 기본 명령어: `git init`, `git add`, `git commit`, `git push`, `git pull`  
2. **WebRTC 개요 복습**  
   - SDP, ICE Candidate, STUN/TURN 등 핵심 용어 정리  
   - P2P 구조 복습: Signaling의 역할과 데이터 교환  
3. **SFU / MCU / Mesh 차이점** (이론 맛보기)  
   - Mesh: 모든 참여자 간 직접 연결  
   - MCU: 서버에서 미디어 디코딩/믹싱 후 재인코딩  
   - SFU: 서버가 단순 포워딩(Selective Forwarding)만 수행

### **실습**
1. **Git 환경 설정**  
   - 자신의 PC(Windows 11 데스크톱)에 Git 설치  
   - GitHub 계정 생성 및 SSH key 등록(필요 시)  
   - 샘플 프로젝트(“HelloGit”) 생성 후 원격 저장소에 push/pull 실습  
2. **P2P WebRTC 간단 복습**  
   - 기존에 만든 P2P WebRTC 예제(혹은 샘플 코드) 확인  
   - ICE Candidate 교환 로직이 어떻게 동작하는지 코드 레벨로 훑어보기

### **숙제(퀴즈)**
1. **Git 개념 퀴즈**  
   - `git clone`, `git pull`의 차이가 무엇인가?  
   - `git add .` 명령어는 무슨 의미를 가지는지 서술하시오.
2. **WebRTC 기본 퀴즈**  
   - P2P 방식에서 SDP와 ICE Candidate가 오고가는 순서를 간단히 요약하시오.
3. **SFU/MCU/Mesh 개념 문제**  
   - SFU 서버의 장점 2가지만 간단히 적으시오.

### **코멘트/팁**
- **Tip**: Git 명령어는 “왜” 사용하는지 의도를 이해해야 실수 줄임.  
- **Tip**: WebRTC의 SDP/ICE는 지금은 복잡해 보여도, 곧 SFU에서 동일하게 활용되므로 꼼꼼히 복습해둘 것.  
- **Day 1**은 강의 전체에서 가장 기초적인 도구와 개념을 다지는 날이니 부담 없이 진행하면 됩니다.

---

## **Day 2**

### **목표**
- SFU 개념 심화 학습 (Mesh 대비 SFU의 장단점 재확인)  
- 오픈 소스 SFU 서버(Janus, mediasoup, Pion 등) 후보 간단 비교  
- Windows 서버 환경에서 SFU를 어떻게 배포할지 큰 그림 잡기

### **이론**
1. **SFU 심화**  
   - SFU가 오디오/비디오 트래픽을 ‘선별적 포워딩’한다는 것이 어떤 의미인지 정리  
   - MCU와 달리 디코딩/인코딩 부담이 클라이언트(발화자/청취자)와 서버 중 어디에 분산되는지 비교
2. **오픈 소스 SFU 서버 후보**  
   - **Janus**: C 언어 기반, 플러그인 구조, 확장성 좋음  
   - **Mediasoup**: Node.js 기반, 높은 성능, JS 커뮤니티 인기  
   - **Pion**: Go 기반, 간단히 시작 가능  
   - (특이사항) Windows에서 빌드/배포 시 호환성 체크
3. **Windows 서버 환경 개념**  
   - IIS / ASP.NET Core / 방화벽 포트 개방 개념  
   - (간단) 네트워크 설정: 사설 IP vs 공인 IP, 포트포워딩

### **실습**
1. **SFU 서버候補 설치 준비**  
   - Janus(또는 Mediasoup) GitHub 리포지토리 방문, 빌드/설치 가이드 확인  
   - Windows 서버 PC에 필요한 의존성(Visual Studio, Node.js 등) 점검  
2. **비교표 작성**  
   - 학생 스스로 “Janus vs Mediasoup vs Pion”을 표로 정리(언어, OS 호환성, 라이선스, 커뮤니티 규모 등)

### **숙제(퀴즈)**
1. **SFU 개념 관련**  
   - “Selective Forwarding”이 구체적으로 어떤 과정을 거치는지 2~3줄로 요약  
2. **오픈 소스 SFU 서버**  
   - Janus와 Mediasoup 중 하나를 선택하고, 자신의 선택 이유를 2가지 이상 써보기
3. **Windows 서버**  
   - 공인 IP가 아닌 사설 IP 환경에서 SFU가 동작하려면 어떤 추가 설정이 필요한가?

### **코멘트/팁**
- **Tip**: SFU 서버 선택은 최종 결정이 아니더라도, 각 서버의 특징을 명확히 비교해두면 이후 구현 시 시행착오가 적어집니다.  
- **Tip**: Windows 서버에 직접 SFU를 돌리는 경우, **방화벽 포트 설정**을 꼭 확인해야 합니다. 향후 WebRTC UDP 포트/ICERest API 포트 등의 개방이 필요합니다.

---

## **Day 3**

### **목표**
- 실제 선택한 SFU 서버(예: Janus) 빌드/설치  
- 간단한 “Hello SFU” 수준 데모 페이지/콘솔 구동  
- 기본 플러그인(혹은 데모 앱)을 활용해 SFU 동작 구조 확인

### **이론**
1. **Janus(예시)**  
   - 구조: Core + Plugins  
   - 주요 Plugin 종류: Echo Test, Video Room, Streaming 등  
   - 설정 파일(janus.cfg, plugins.cfg) 개념과 기본 파라미터
2. **SFU 서버 동작 원리**  
   - ICE, DTLS, SRTP 처리 흐름(서버 관점)  
   - 다수 클라이언트가 접속할 때 방(Room) 구조가 어떻게 되는지(일단 기본적인 handle/Session 개념)

### **실습**
1. **Windows 서버에서 Janus 빌드/설치** (예시)  
   - Git clone → cmake or Visual Studio로 빌드  
   - 의존 라이브러리(OpenSSL, libnice 등) 설치  
   - 콘솔에서 `janus --help` 실행해보기  
2. **데모 테스트**  
   - Janus 제공 공식 데모 페이지(예: EchoTest) 접속  
   - 로컬/원격 웹 브라우저에서 오디오/비디오 연결 시도, SFU 동작 로그 확인  
   - 로그에서 ICE Candidate 수락, DTLS handshake, 오디오 스트림 확인

### **숙제(퀴즈)**
1. **Janus 설정**  
   - `janus.cfg` 파일에서 로그 레벨(log_level)이나 다른 파라미터를 조정하면 어떤 효과가 있는지 알아보기  
2. **실습 문제**  
   - 에코 테스트(“Echo Test Plugin”)에서 본인이 말한 음성이 잘 돌아오는지 확인하고, 마이크/스피커를 바꿔보고 로그 변화 관찰  
3. **이슈 트래킹**  
   - 빌드 도중 발생한 문제(에러/경고)가 있었다면, GitHub Issue나 Stack Overflow 등에서 해결책을 찾아보고 간단히 정리

### **코멘트/팁**
- **Tip**: 설치 과정에서 에러가 발생해도, **오류 메시지**를 검색하면 대부분 이미 해결법이 나와 있습니다.  
- **Tip**: EchoTest 성공 = SFU 서버가 정상 구동되었음을 의미. 이걸 기점으로 안드로이드와 연동할 발판이 마련됩니다.  
- **Tip**: 다른 SFU(Mediasoup, Pion)를 선택했다면, 그에 맞춰 비슷한 실습 과정을 밟으면 됩니다.

---

## **Day 4**

### **목표**
- SFU 서버의 내부 동작(Plugin/모듈) 구성 이해  
- 서버 로그 분석 / 간단한 커스텀 설정 변경  
- 향후 Android 클라이언트 연동을 대비해, **시그널링** 구조 살펴보기

### **이론**
1. **Janus Plugin 구조(예시)**  
   - VideoRoom, AudioBridge 등 주요 plugin 소개  
   - 각 Plugin이 어떤 포트를 사용하는지, 어떤 API(REST/WebSocket)를 제공하는지  
2. **시그널링 방식**  
   - WebSocket vs HTTP REST vs 다른 프로토콜  
   - WebRTC 연결 과정에서 서버가 Offer/Answer를 어떻게 주고받는지(예시: Janus JSON RPC)

### **실습**
1. **Plugin 설정 커스터마이징**  
   - 예: `videoroom.jcfg`(VideoRoom plugin 설정)에서 max_bitrate, audio_level_event 등 파라미터 살펴보기  
   - 간단히 값을 바꿔보고 SFU 서버 재시작 → 서버 로그 확인  
2. **간단한 API 호출**  
   - Postman 등 툴로 Janus Admin API(REST) 호출: “list of sessions/handles” 조회  
   - SFU 동작을 API 레벨에서 모니터링

### **숙제(퀴즈)**
1. **Plugin 구조 퀴즈**  
   - VideoRoom과 AudioBridge의 차이점을 짧게 정리  
2. **시그널링**  
   - WebSocket 시그널링과 HTTP REST 시그널링의 장단점을 비교해보기  
3. **API 사용**  
   - Janus Admin API를 통해 세션 현황을 확인하려면 어떤 Endpoint를 호출해야 하는지?

### **코멘트/팁**
- **Tip**: SFU가 단순히 “미디어 전달”만 하는 게 아니라, “Room 생성/관리”나 “오디오 레벨 모니터링” 같은 부가 기능을 Plugin 단에서 할 수 있습니다.  
- **Tip**: 향후 Android 앱에서 이 **시그널링 API**를 직접 호출할 수도 있고, 중간 서버(예: Node.js, Firebase, Socket.io)로 감싸서 연동할 수도 있습니다.

---

## **Day 5**

### **목표**
- Week 1 전체 내용 정리 및 **다음 주차(안드로이드 연동) 준비**  
- Git 브랜치 전략(메인/개발 등) 적용, 간단한 협업 시나리오 체험  
- SFU 서버(Windows)에서 기본 구동 스크립트/서비스화 시도

### **이론**
1. **Git 협업 전략**  
   - Branch naming(예: `main`, `develop`, `feature/...`)  
   - 간단한 Pull Request(review) 프로세스 소개  
2. **Windows 서비스 등록**  
   - Windows에서 SFU 서버를 백그라운드 서비스처럼 띄우는 법(간단 소개)  
   - 혹은 스크립트 배치파일 작성 후 자동 실행 설정

### **실습**
1. **Git 브랜치 실습**  
   - 기존 “SFU-Project” 저장소에 `develop` 브랜치 생성 → `feature/week1` 브랜치 만들어 Day5 실습 코드/설정 반영  
   - Pull Request, Merge, Conflict 해결 연습  
2. **SFU 서버 자동 구동**  
   - `startup.bat`(배치파일) or Windows Scheduler를 사용해 부팅 시 자동 실행 설정  
   - 서버 로그가 잘 수집되는지 로그 파일 경로 설정 확인

### **숙제(퀴즈)**
1. **Git Flow 퀴즈**  
   - `main`(또는 `master`)와 `develop` 브랜치의 차이점, 왜 분리해서 쓰는지  
2. **Windows 서비스 등록**  
   - SFU 서버를 실서비스로 운영하려면, 지속 실행 + 재시작 시 자동 구동이 필요한 이유를 설명  
3. **Week 2 대비 질문**  
   - 안드로이드 앱과 SFU 서버를 어떻게 연동할지, 시그널링 경로(직접 vs Firebase vs Socket.io) 아이디어를 짧게 서술

### **코멘트/팁**
- **Tip**: 강의 중에는 1인 개발 방식이지만, Git 브랜치를 통해 여러 명이 동시에 작업하는 연습을 미리 해두면 나중에 협업에 큰 도움이 됩니다.  
- **Tip**: SFU 서버가 항상 켜져 있어야 클라이언트가 접속 가능하므로, Windows 서비스를 구성하거나 Docker로 띄우는 방안도 추후 고려할 수 있습니다.  
- **Tip**: Day 5 이후에는 안드로이드 환경에서 본격적으로 SFU를 연동할 예정이니, 다음 주차 내용(Week 2 Day 1~5)와 연결해서 복습해주세요.

---

# 마무리 정리

- **Week 1**은 **Git/GitHub 기초**, **SFU 개념 학습**, **오픈 소스 SFU 서버 설치·테스트**까지를 목표로 합니다.  
- 이 과정을 모두 마치면, **SFU 서버가 정상적으로 구동**되고, **기초적인 API/Plugin** 작동을 확인할 수 있어, **Week 2**에서 안드로이드 클라이언트와 연동하는 발판이 마련됩니다.  
- 각 Day마다 제시된 숙제(퀴즈)는 실제 코드/설정 파일에 손을 대보고, 개념을 짧게 정리하는 데 초점을 두었습니다.

이상으로 **Week 1 Day 1~5**의 상세 강의안이었습니다. 다음 주차(Week 2)도 동일한 형식으로 요청해주시면, 이어서 안드로이드 연동 및 멀티 유저 연결을 다루는 세부 계획을 작성해 드리겠습니다!