아래는 **Week 2 (Day 1 ~ Day 5)**의 상세 강의안입니다.  
이전 주차(Week 1)에서 **Janus SFU 서버를 설치 및 기본 테스트**를 마쳤다고 가정하고,  
이제 **Android 클라이언트와 SFU 서버 간 1:1 음성 전송**을 목표로 하는 단계입니다.

포맷은 이전과 동일하게 **“목표 / 이론 / 실습 / 숙제(퀴즈)”**이며, 마지막에 **Day별 코멘트(팁)** 섹션을 추가했습니다.

---

# **Week 2: SFU 서버 & 안드로이드 클라이언트 최소 연동**

## **Day 1**

### **목표**
- 안드로이드 스튜디오 프로젝트 생성 및 WebRTC 라이브러리 세팅  
- Janus(또는 WebRTC)와 연동할 **시그널링 구조**(웹소켓/REST) 개략 설계  
- 기초 UI(버튼, 텍스트 필드) 구성

### **이론**
1. **Android WebRTC 라이브러리 개요**  
   - Google 공식 WebRTC 라이브러리(aar) 혹은 Maven 의존성  
   - 주요 클래스: `PeerConnectionFactory`, `PeerConnection`, `MediaConstraints` 등  
2. **시그널링(SDP/ICE) 개념 복습**  
   - P2P 때와 달리, 이제는 **SFU 서버**(Janus)가 Offer/Answer를 주고받는 상대  
   - WebSocket vs REST: Android 측에서 Janus를 호출하는 기본 시나리오  
3. **안드로이드 권한**  
   - 오디오 녹음(`RECORD_AUDIO`), 네트워크 권한  
   - AndroidManifest, 런타임 퍼미션 체크

### **실습**
1. **프로젝트 생성**  
   - Android Studio에서 새 프로젝트 “SFUClient” 생성 (Kotlin 기반 권장)  
   - 최소 SDK 설정, 권한 추가(오디오, 인터넷 등)  
2. **WebRTC 라이브러리 세팅**  
   - `build.gradle`에 WebRTC 의존성 추가 (예: `org.webrtc:google-webrtc:x.x.x`)  
   - 간단히 컴파일 및 빌드가 완료되는지 확인  
3. **간단 UI 구성**  
   - `activity_main.xml`에 “Connect” 버튼 / “Disconnect” 버튼 / 로그 출력용 `TextView` 정도 배치  
4. **시그널링 구조 설계 문서**  
   - Janus Docs(https://janus.conf.meetecho.com/)에서 웹소켓/REST 시그널링 예제 확인  
   - 다음 Day에 본격적으로 구현할 준비

### **숙제(퀴즈)**
1. **WebRTC 라이브러리**  
   - `PeerConnectionFactory`의 역할을 한 문장으로 요약  
2. **권한**  
   - Android에서 “런타임 퍼미션”과 “Manifest 권한”이 다른 점은 무엇인가?  
3. **시그널링**  
   - Janus와 연동하기 위해 사용할 시그널링 프로토콜(REST or WebSocket)을 선택하고, 그 이유를 짧게 서술

### **코멘트/팁**
- **Tip**: WebRTC aar(또는 Maven) 버전에 따라 호환성 문제가 있을 수 있으니, 릴리즈 노트나 예제 프로젝트를 미리 참고하세요.  
- **Tip**: 아직은 서버와 직접 연결하기 전 단계이므로, 프로젝트 구조와 권한 설정에 집중합니다.  
- **Tip**: Day 1~2 정도에서 시그널링 로직을 구상해두면 이후 개발 속도가 빨라집니다.

---

## **Day 2**

### **목표**
- 안드로이드 앱에서 Janus에 **Offer** 보내고, **Answer** 받아오는 흐름 구현(1:1 음성 연결)  
- WebSocket 또는 REST 기반 시그널링 로직 코드 작성 시작  
- `PeerConnection` 초기화 및 세션 설정

### **이론**
1. **Janus 시그널링 프로세스**  
   - Janus JSON API: `create` → `attach` → plugin 메시지 교환 → `offer` → `answer` 등  
   - 트랜잭션(특정 요청과 응답 매핑) 개념  
2. **PeerConnection 초기화**  
   - `PeerConnectionFactory.createPeerConnection()`  
   - ICE 서버 설정(STUN/TURN), `MediaConstraints`  
3. **오디오 트랙(MediaStreamTrack) 구성**  
   - 마이크 캡처 → Opus 인코딩 → RTP 전송

### **실습**
1. **시그널링 코드**  
   - 안드로이드 측에서 Janus WebSocket/REST 서버에 접속  
   - “세션 생성” → “플러그인(예: AudioBridge / VideoRoom)”에 attach → Offer 전송  
   - Janus에서 “Answer”를 수신하면, `PeerConnection.setRemoteDescription()` 호출  
2. **PeerConnection 객체 생성**  
   - `PeerConnectionFactory`, `AudioSource`, `AudioTrack` 설정  
   - ICE 서버: Windows 서버 IP(또는 STUN/TURN) 정보 기입  
3. **UI 연동**  
   - “Connect” 버튼을 누르면 시그널링 → Offer/Answer 교환  
   - 성공 시 로그에 “Connected!” 등 표시

### **숙제(퀴즈)**
1. **Janus API**  
   - 세션 생성 후 플러그인을 attach하기 위해 어떤 JSON 명령을 보내야 하는지? (메시지 예시 작성)  
2. **PeerConnection**  
   - `PeerConnection` 생성 시, 꼭 필요한 파라미터 3가지(예: ICE 서버, Constraints 등)를 적어보시오.  
3. **AudioTrack**  
   - `AudioSource`와 `AudioTrack`의 관계를 간단히 정리

### **코멘트/팁**
- **Tip**: 시그널링 로직에서 “transaction” 개념(요청 → 응답)이 중요합니다. Janus는 JSON 메시지마다 고유의 transaction ID를 사용하여 응답을 매핑합니다.  
- **Tip**: Offer/Answer를 교환할 때, `createOffer()` → `onCreateSuccess(...)` → `setLocalDescription(...)` → 시그널링 서버 전달 → 이후 Answer 받으면 `setRemoteDescription(...)` 순서를 꼭 지켜야 합니다.

---

## **Day 3**

### **목표**
- **1:1 오디오 연결** 실제 테스트(스마트폰 ↔ Janus ↔ Echo plugin or AudioBridge)  
- ICE 연결 상태, DTLS handshake 상태 등 **로그** 확인  
- 오디오 권한 및 마이크 동작 문제 해결

### **이론**
1. **ICE & DTLS**  
   - ICE Candidate 발견/교환 절차, 리플렉티드 주소, Relay 주소 개념  
   - DTLS handshake 시점(연결 성립 전), 보안 통신 여부  
2. **오디오 입출력**  
   - 안드로이드에서 마이크 입력(캡처) → WebRTC stack → 네트워크 전송  
   - 수신된 오디오(Opus) → 디코딩 → 스피커 플레이

### **실습**
1. **1:1 오디오 통신 시연**  
   - 안드로이드 폰(또는 에뮬레이터)에서 “Connect” → Janus SFU → Echo/AudioBridge Plugin → 음성 수신  
   - 실제로 말하면 에코가 들리거나, AudioBridge의 다른 참가자(테스트 클라이언트)와 통화 가능  
2. **로그 모니터링**  
   - Janus 콘솔 로그: ICE gathering, DTLS handshake 완료, room join 메시지  
   - Android Logcat: `PeerConnectionObserver` 콜백(`onIceCandidate`, `onIceConnectionChange`, etc.)  
3. **권한 이슈 해결**  
   - 마이크 권한을 거부했을 때 처리(Toast나 Alert), 연결 시 에러 로그 확인  
   - 스피커/헤드폰 변경 테스트

### **숙제(퀴즈)**
1. **ICE Candidate**  
   - ICE gathering이 “complete” 되기까지 어떤 단계가 있는지 2~3줄로 설명  
2. **오디오 체크**  
   - “에코가 들린다”는 것은 어떤 plugin 테스트인지? (EchoTest vs AudioBridge)  
3. **디버깅**  
   - 연결 안 될 때 확인해야 할 3가지 포인트(예: 방화벽, 권한, Janus 플러그인 설정)를 적어보시오.

### **코멘트/팁**
- **Tip**: 에뮬레이터로 테스트할 때 마이크가 제대로 작동하지 않을 수 있으므로, 실제 디바이스 테스트를 병행하면 좋습니다.  
- **Tip**: `onIceConnectionChange(FAILED)` 등 상태 변화를 관찰하며, 어디서 문제가 발생했는지 추적하는 습관을 들이세요.

---

## **Day 4**

### **목표**
- **디버깅 / 에러 처리** 방안 숙지  
- 다양한 네트워크 환경(사설 IP, 공인 IP)에서 SFU 연결 시도 → 문제 해결  
- 안드로이드 UI 개선(상태 표시, 연결 해제 버튼 등)

### **이론**
1. **NAT/방화벽 이슈**  
   - STUN vs TURN 차이, Windows 서버에 TURN 서버 설치가 필요한 이유  
   - IPv4, IPv6 혼재 환경에서의 이슈  
2. **안드로이드 UI/UX**  
   - 연결 상태 표시: “Connecting...”, “Connected”, “Disconnected” 등  
   - 에러 시 사용자에게 안내(Toast/Alert)

### **실습**
1. **네트워크 테스트**  
   - 다른 Wi-Fi 망, 휴대폰 데이터 망(LTE/5G)에서 접속 시도  
   - TURN 서버(예: coturn) 설정 후 `PeerConnection` ICE 서버 목록에 추가  
   - 로그 상에서 “srflx”, “relay” 타입이 보이는지 확인  
2. **에러 처리**  
   - 연결 실패 시 재시도 버튼 or 메시지 표시  
   - 강제로 서버를 끄거나 방화벽 포트 막아본 뒤, 앱 측 반응 확인  
3. **UI 개선**  
   - 연결 상태 텍스트뷰, 해제(Disconnect) 버튼 동작 구현  
   - 오디오 음소거(Mute) 버튼(마이크 On/Off) 간단히 추가

### **숙제(퀴즈)**
1. **STUN vs TURN**  
   - TURN이 필요한 상황을 예시 들어 설명  
2. **UI/UX**  
   - 연결 상태가 “Disconnected”가 되면, 사용자는 어떤 액션을 할 수 있어야 하는가?  
3. **방화벽**  
   - Windows 서버에서 특정 UDP 포트를 열어야 하는 이유를 한 문장으로 요약

### **코멘트/팁**
- **Tip**: 실제 투어 시스템을 염두에 두면, 모바일 환경에서 NAT 문제는 매우 흔합니다. STUN/TURN 설정을 꼼꼼히 해두면 안드로이드 장치가 어디서든 접속 가능해집니다.  
- **Tip**: UI/UX는 간단해도 좋지만, 상태 표시가 잘 되어 있으면 디버깅이 훨씬 수월해요.

---

## **Day 5**

### **목표**
- Week 2 전체 마무리: “**안드로이드 ↔ Janus SFU 1:1 음성 통신**” 프로토타입 완성  
- Git 버전 태깅(`v0.1-1to1-audio-test` 등), 코드 리뷰 & 문서화  
- 다음 주차(멀티 Room 및 다자 연결) 준비

### **이론**
1. **프로토타입 구조** 정리  
   - 안드로이드 앱(시그널링 + WebRTC) ↔ Janus SFU(Plugin) ↔ (에코 혹은 타 클라이언트)  
2. **향후 멀티 Room 확장** 개요  
   - Janus `VideoRoom` or `AudioBridge`로 다자 연결  
   - 안드로이드 앱에서 “Room ID” 입력 / 초대 기능 / 음소거 제어 등 추가 가능

### **실습**
1. **최종 1:1 통신 데모**  
   - 친구(또는 동료) 기기에 앱 설치 → 서로 다른 계정(또는 같은 방)에서 음성 통화  
   - Janus 콘솔 로그로 상태 모니터링  
   - 실제 대화 품질(지연, 잡음 등) 확인  
2. **Git 릴리스 태깅 & 문서화**  
   - 프로젝트 README에 “세팅 방법”, “빌드 방법”, “연결 방법” 간단히 기술  
   - 버전 태그: `git tag -a v0.1-audiotest -m "1:1 Audio Test Completed"`  
   - Push tags → GitHub에 버전 릴리스 생성  
3. **코드 리뷰**  
   - 주요 함수: 시그널링 부분, `PeerConnectionObserver` 구현부, `onIceCandidate()` 처리 등

### **숙제(퀴즈)**
1. **프로토타입 구조**  
   - 안드로이드와 Janus 사이에 실제로 오가는 메시지(Offer/Answer)의 개수를 나열하고, 언제 발생하는지 서술  
2. **Git 릴리스**  
   - Git에서 tag를 만드는 명령어(`git tag`)와 push하는 명령어(`git push --tags`)를 실제 예시로 작성  
3. **멀티 Room 확장**  
   - 다음 주차에 “Room 개념”을 구현할 때, 어떤 추가 데이터(예: Room ID, 참가자 리스트)가 필요할지 생각해보고 간단히 정리

### **코멘트/팁**
- **Tip**: 1:1 오디오 연결이 제대로 된다면, SFU를 통한 기본 음성 통신 파이프라인은 완성된 셈입니다. Week 3부터는 이를 “다자 연결(방 구조)”로 확장해가면 됩니다.  
- **Tip**: 항상 Git에 상세 커밋 메시지와 태그를 남겨두면, 나중에 버전을 되돌리거나 비교할 때 무척 편리합니다.  
- **Tip**: 이번 주차에서 네트워크 이슈(방화벽, NAT)와 안드로이드 권한 이슈(마이크, 오디오)만 확실히 해결해두면, 멀티 연결 구현 시 많은 문제를 사전에 피할 수 있습니다.

---

# 요약

- **Week 2**는 **Android ↔ Janus SFU간 1:1 음성 통신**을 실현하는 데 집중합니다.  
- Day 1~2에서 **프로젝트 세팅 + 시그널링 로직**을 작성하고, Day 3~4에서 **실제 1:1 통신 테스트 & 디버깅**을 진행, Day 5에 **프로토타입 마무리 및 Git 태깅**을 수행합니다.  
- 이를 통해 학생들은 “Android WebRTC 코드 흐름 + Janus 시그널링 API”를 **실전 수준**으로 체득하게 됩니다.  
- 다음 주(Week 3)부터는 **Room 개념**, **다자 연결**, **오디오 품질 튜닝** 등을 다루며, 투어 가이드 시스템 같은 형태로 확장할 준비를 갖추게 됩니다.

이상으로 **Week 2 Day 1~5**의 상세 강의안이었습니다. 필요하다면 이후 주차(Week 3) 강의안도 동일한 형식으로 이어서 요청해 주시면, 멀티 유저(방) 구조 및 안정화 과정을 자세히 작성해드리겠습니다!