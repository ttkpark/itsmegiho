# Week 1 Day 3 – Janus SFU 빌드 및 설치, 데모 페이지 구동

안녕하세요! **Week 1 Day 3** 수업 자료입니다.  
지난 시간(Week 1 Day 2)까지는 **SFU 개념 심화**, **오픈소스 SFU 비교**, **Windows 서버 배포 전략**을 살펴봤습니다.  
오늘(Week 1 Day 3)에는 실제로 **Janus SFU를 빌드 및 설치**하고, **데모 페이지 / 기본 플러그인**을 구동해  
SFU가 정상 동작하는지 **직접 확인**해보겠습니다.

> **전제**  
> - Week 1 Day 1~2에서 Git/GitHub를 사용해 학습 문서를 남기는 습관을 들였고, SFU 개념 및 Janus 서버 개요를 파악함.  
> - 이제 실제 **Windows 서버**(또는 Windows 11 데스크톱)에 **Janus**를 설치해보거나,  
>   WSL/Docker 등 편한 환경에서 **기본 데모(EchoTest, VideoRoom, AudioBridge)**를 실행해볼 예정.

---

## 1. 목표

1. **Janus 빌드 & 설치**  
   - Windows 환경에서 Janus를 빌드하는 절차를 경험 (WSL 사용 권장, 또는 Docker)  
   - 필요한 의존성(libnice, libsrtp, libwebsockets 등) 확인

2. **데모 페이지 / 플러그인 구동**  
   - Janus가 기본 제공하는 **EchoTest**, **VideoRoom**, **AudioBridge** 등의 테스트 페이지 접속  
   - 브라우저에서 오디오/비디오가 실제 SFU를 통해 포워딩되는지 확인

3. **SFU 동작 확인 & 로그 분석**  
   - Janus 콘솔 로그에서 ICE/DTLS/오디오/비디오 설정이 제대로 이루어지는지 모니터링  
   - 이후 안드로이드 앱과 연동하기 전, 서버 쪽이 정상 작동함을 확실히 검증

---

## 2. 이론

### 2.1 Janus 빌드(Windows & WSL) 개념

1. **Windows 빌드 시 고려사항**  
   - **WSL(Windows Subsystem for Linux)**를 통해 Ubuntu 환경에서 빌드하는 방안이 일반적으로 권장됨  
   - 직접 Cygwin/MinGW로 빌드 가능하나, 의존 패키지 설치가 까다로울 수 있음  
   - **Docker**로 빌드(리눅스 컨테이너) 후, Windows에서 실행하는 방법도 있음

2. **주요 의존성**  
   - `libnice`: ICE/STUN/TURN 기능 제공  
   - `libsrtp`: SRTP(미디어 암호화)  
   - `openssl`: TLS/DTLS  
   - `libmicrohttpd`, `libwebsockets`: HTTP/WS 지원  
   - Janus “plugins” 구동을 위한 추가 라이브러리(예: Opus, libvpx 등)

### 2.2 데모 플러그인

1. **EchoTest**  
   - 가장 단순한 테스트. 브라우저에서 마이크/카메라 허용 → 자기 자신에게 돌아오게 하여 미디어 경로 확인  
2. **VideoRoom**  
   - 다수 사용자 간 비디오/오디오 회의가 가능한 메인 플러그인  
3. **AudioBridge**  
   - 오디오 전용 회의 테스트 플러그인

> 이 데모들을 통해 **ICE / DTLS 핸드셰이크**가 정상 동작하는지 손쉽게 검사할 수 있음.

### 2.3 SFU 동작 확인 요소

- **Janus 콘솔 로그**: ICE Gathering, DTLS Handshake, SRTP 암호화 등 단계별 메시지  
- **브라우저 콘솔**: getUserMedia 성공 여부, PeerConnection 이벤트  
- **네트워크/방화벽**: UDP 포트 열려 있는지 확인

---

## 3. 실습

이번 실습에서는 **WSL(Ubuntu)** 상에서 빌드하는 방법을 예로 들지만,  
**Docker**나 **Cygwin**으로 빌드를 시도해도 됩니다.  
(어떤 방법을 택하든, Git으로 **실습 기록**을 남기는 것이 목표입니다.)

### 3.1 Git 브랜치 생성

1. **새 브랜치**  
   - Week 1 Day 3 작업을 위한 브랜치:
     ```bash
     git checkout main
     git pull
     git checkout -b week1-day3
     ```
   - 출력 예: `Switched to a new branch 'week1-day3'`
2. **docs 폴더** (이미 있을 가능성 있음)  
   - `cd docs`
   - `touch Janus-Build.md` (or use VSCode to create the file)  
3. **간단 문서 템플릿 작성**  
   ```markdown
   # Janus Build & Test (Week 1 Day 3)

   ## WSL Installation Steps
   1. ...

   ## Dependencies
   - libnice, libsrtp, openssl, ...

   ## Build & Run
   - ...

   ## Demo Tests
   - EchoTest, VideoRoom plugin
   ```
   - 저장 후 커밋:
     ```bash
     git add .
     git commit -m "Add Janus-Build.md for Day3"
     ```

### 3.2 WSL 환경에서 Janus 빌드

> **(인터넷 검색)**  
> “Janus Windows WSL build” / “Janus official documentation” 검색  
> → [https://janus.conf.meetecho.com/docs/index.html](https://janus.conf.meetecho.com/docs/index.html)  
> → 빌드 가이드/의존성 설치 방법 확인

1. **WSL(Ubuntu) 설치**  
   - Windows “앱 & 기능” → “프로그램 및 기능” → “Windows 기능 켜기/끄기” → “Windows Subsystem for Linux” 체크  
   - Microsoft Store에서 “Ubuntu 20.04 LTS” 등 설치  
   - 설치 후, Ubuntu 터미널을 열어서 업데이트:
     ```bash
     sudo apt-get update
     sudo apt-get upgrade
     ```
2. **의존성 설치**  
   - 예시 (일부 라이브러리만 예시이므로 최신 문서 참고):
     ```bash
     sudo apt-get install libmicrohttpd-dev libjansson-dev \
       libssl-dev libsofia-sip-ua-dev libglib2.0-dev libopus-dev \
       libogg-dev libcurl4-openssl-dev liblua5.3-dev libconfig-dev \
       pkg-config gengetopt libtool automake autoconf \
       libnice-dev libsrtp2-dev libwebsockets-dev cmake
     ```
   - 설치가 오래 걸릴 수 있으니 기다림
3. **Janus 소스 다운로드**  
   ```bash
   git clone https://github.com/meetecho/janus-gateway.git
   cd janus-gateway
   git checkout v1.0.0  # 예시로 특정 버전 체크아웃 (또는 main)
   ```
4. **빌드**  
   ```bash
   sh autogen.sh
   ./configure --prefix=/opt/janus
   make
   sudo make install
   sudo make configs
   ```
   - 이 과정을 통해 `/opt/janus` 아래에 실행 파일과 설정 파일이 배포됨.
   - 빌드 로그에 에러가 있는지 확인  
   - 성공 시 `ls /opt/janus/bin/`에서 `janus` 실행파일을 볼 수 있음

### 3.3 Janus 서버 구동 & 데모 페이지 테스트

1. **Janus 실행**  
   - `/opt/janus/bin/janus`:
     ```bash
     cd /opt/janus/bin
     ./janus
     ```
   - 콘솔에 “Reading configuration from janus.cfg” 같은 메시지 출력, 플러그인 로드  
   - 만약 Windows 호스트 브라우저에서 접속하려면, WSL IP(또는 localhost proxy) 확인

2. **EchoTest 데모**  
   - Janus 기본 설정에 따라, `http://<WSL_IP>:8088/` (또는 `http://localhost:8088/`)로 접속  
   - “Echo Test” 링크 클릭 → 카메라/마이크 허용 → 브라우저에서 **본인 음성/영상**이 다시 재생되는지 확인  
   - 콘솔(Log)에 ICE/DTLS handshake 로그 출력 확인
3. **VideoRoom / AudioBridge**  
   - “VideoRoom test” 클릭: 다수 인원(브라우저 탭 여러 개) 접속 가능  
   - “AudioBridge test”: 오디오 전용 회의 시뮬레이션  
   - 동작 잘 되면 SFU 포워딩 성공

4. **Git 문서 업데이트**  
   - 빌드/실행 과정, 에러 해결 과정을 `docs/Janus-Build.md`에 기록:
     ```markdown
     ## Running Janus
     - cd /opt/janus/bin
     - ./janus
     - Access on http://localhost:8088/ for demos
     ```
   - 커밋:
     ```bash
     git add .
     git commit -m "Document Janus demo test steps"
     ```

### 3.4 로그 분석 & 문제 해결

1. **Janus 콘솔 로그**  
   - ICE candidates, DTLS handshake status(“dtls_srtp: incoming request...”), SRTP negotiation  
   - Error message 예: 포트 충돌, 인증 관련  
2. **브라우저 콘솔**  
   - Media permission(마이크/카메라) 허용 안 했을 때 발생하는 에러  
   - ICE connection failed(방화벽 문제)  
3. **Windows Defender Firewall**  
   - 8088, 8188, 8089(TLS) 등 필요한 포트 열려 있는지 점검  
   - UDP 포트(10000 이상)도 열어야 실전에서 타 기기가 접속 가능

---

## 4. 숙제(퀴즈)

1. **Janus 빌드 퀴즈**  
   - 빌드시 필요한 주요 라이브러리(libnice, libsrtp, openssl 등)가 각각 어떤 역할을 하는지 간단히 작성  
2. **데모 플러그인**  
   - EchoTest와 VideoRoom의 차이를 1~2줄로 요약  
3. **오류 대처**  
   - `./janus` 실행 시 “Couldn’t open config file janus.cfg” 에러가 뜬다면 어떻게 해결할 수 있을까?

(숙제 답안을 `docs/Day3-quiz.md` 등으로 정리한 뒤, `git commit & push` 해주세요!)

---

## 5. 코멘트/팁

1. **WSL Network**  
   - WSL2는 기본적으로 NAT 형태이므로, 외부에서 접속하려면 `wsl --ipaddress` 등의 명령어로 IP를 찾거나,  
     Windows Host와 포트포워딩 설정이 필요할 수 있음.  
   - Docker도 마찬가지로 브릿지 네트워크 사용 시 포트 매핑 필수.

2. **실행 스크립트 / 자동화**  
   - 매번 `/opt/janus/bin/janus` 실행하기 번거롭다면,  
     `janus.service`를 만들어 systemd 서비스로 등록하거나  
     배치파일(Windows)로 자동화하는 방법도 좋음.

3. **Day 3 의의**  
   - 안드로이드와 연동하기 전, **Janus 서버가 제대로 동작**하는지 ‘브라우저 데모’로 확인했다는 점이 매우 중요.  
   - ICE, DTLS, SRTP 가 정상 작동하는지 미리 검증하면, 이후 Mobile 테스트 시 문제 범위를 좁힐 수 있음.

4. **다음 수업 예고**  
   - **Day 4**부터는 안드로이드에서 Janus와 1:1 음성 통신을 시도 (Offer/Answer, ICE candidate 교환)  
   - 여기서 “Room 개념(예: VideoRoom Plugin)”을 안드로이드 앱에서 어떻게 시그널링할지 설계/구현하게 됨.

---

# 마무리

오늘 **Week 1 Day 3** 수업을 모두 따라 했다면,

- [ ] **Janus 빌드/설치**(WSL/Docker/Cygwin 중 택1) 성공  
- [ ] **EchoTest / VideoRoom** 데모 페이지 접속, 오디오/비디오 확인  
- [ ] **Git**으로 빌드 과정/이슈 해결 과정을 문서화 & 커밋  
- [ ] Windows 방화벽, 포트 설정 등을 꼼꼼히 체크  

이 모든 것을 달성하셨을 것입니다.  
이제 Janus SFU 서버가 실제로 동작함을 확인했으니,  
곧 안드로이드 앱에서 **Offer/Answer**를 주고받고, **음성 연결**을 시도하는 **다음 단계**로 넘어갈 준비가 되었습니다!
