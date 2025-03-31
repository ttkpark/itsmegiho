아래에는 **Week 1 ~ Week 2의 요약**을 간단히 정리하고, 이어서 **Week 3 Day 1~5**의 상세 강의안을 제시합니다.  
Week 3는 **멀티 유저(Room) 구조와 다자 연결**, **오디오 품질 튜닝**에 중점을 두며, **가이드(방 생성) + 여러 참가자(동시 음성 통신)**가 가능한 프로토타입을 목표로 합니다.

---

# **[요약] Week 1 ~ Week 2**

## **Week 1 요약**  
- **Day 1~2**:  
  - Git/GitHub 기초, WebRTC 기본(P2P, SDP, ICECandidate) 복습  
  - SFU 개념(Mesh/MCU/SFU 차이) 학습, 오픈 소스 SFU(Janus/Mediasoup) 비교  
- **Day 3~5**:  
  - Janus SFU 서버 설치/빌드(Windows 환경), 기본 Demo(에코 테스트 등) 시연  
  - Plugin/시그널링 구조 파악, 서버 로그 분석  
  - Git 브랜치 전략, Windows 서비스(자동 구동) 설정 등

> **결과**:  
> - Janus SFU가 로컬/원격 서버에서 정상 구동됨을 확인  
> - EchoTest/AudioBridge plugin으로 미디어가 오가는지 테스트 완료  
> - Git/GitHub로 프로젝트 버전 관리 시작

## **Week 2 요약**  
- **Day 1~2**:  
  - Android 스튜디오에 **WebRTC 라이브러리** 세팅  
  - Janus 시그널링(REST/WebSocket) 구조 설계, 기초 UI(버튼, 로그) 구성  
  - PeerConnection 초기화(Offer/Answer 교환 로직, ICE 서버 설정)  
- **Day 3~4**:  
  - 1:1 오디오 연결 실습(안드로이드 ↔ Janus SFU ↔ Plugin)  
  - ICE/DTLS 및 권한(마이크) 이슈 해결, 에러 처리(연결 실패 시 재시도, 로그 확인)  
  - 네트워크 환경(사설 IP, NAT)에서 TURN 서버 설정  
- **Day 5**:  
  - 1:1 오디오 프로토타입 완성, Git 태깅(`v0.1-audiotest`) & 문서화  
  - 멀티 유저 확장을 위한 기초 논의(Room 개념, AudioBridge/VideoRoom Plugin)

> **결과**:  
> - 안드로이드 클라이언트 + Janus SFU를 통한 **1:1 음성 통신** 프로토타입 완성  
> - 시그널링, Offer/Answer, ICE Connection 로직을 안드로이드에서 직접 구현/디버깅  
> - 멀티 유저/Room 기반 구현에 대비한 기초 지식 습득

---

# **Week 3: 멀티 유저(Room) 구조 & 다자 연결**

## **Day 1**

### **목표**
- “Room(방)” 개념을 본격적으로 도입해 **멤버 관리**(가이드, 참여자)를 할 수 있는 구조 설계  
- Janus에서 `VideoRoom`(혹은 `AudioBridge`) Plugin을 활용해 **다자 연결** 기본 로직 파악  
- 안드로이드 앱에서 “방 만들기/방 참여” UI 기획

### **이론**
1. **다자 연결(Multi-Party) 구조**  
   - 단순 1:1 → N명의 사용자(가이드 + 참여자) 오디오 스트림 관리  
   - Server(SFU) 관점: Room ID, Publisher/Subscriber 개념, 각 Subscriber에게 필요한 스트림 포워딩  
2. **Janus VideoRoom or AudioBridge**  
   - VideoRoom: 비디오/오디오 동시 가능, Room 개념 내장  
   - AudioBridge: 오디오 전용, 다자 음성 회의에 특화  
   - 방 생성, 참가자 등록(Attach), 참가자 목록 관리(Join/Leave 이벤트)

### **실습**
1. **Room 구조 설계 문서 작성**  
   - 안드로이드 앱: “방 생성” → Janus에 “create room” 요청(또는 자동 생성 모드)  
   - 방 ID, 참가자 Nickname(가이드/청취자) 같이 전달  
2. **간단 UI 설계**  
   - 방 생성 버튼, 방 참여 버튼, 방 ID 입력창, 참가자 목록 표시(간단)  
   - 기능: “Create Room” → 서버 응답으로 방 ID 받기 → 앱에 표시  
3. **Janus Config 확인**  
   - `videoroom.jcfg` 또는 `audiobridge.jcfg`에서 room auto-create 옵션, max publishers 설정 등 확인

### **숙제(퀴즈)**
1. **방(Room) 개념**  
   - SFU에서 “Room”을 식별하기 위해 필요한 최소 정보 2가지를 적으시오(예: 방 ID, Plugin ID 등).  
2. **VideoRoom vs AudioBridge**  
   - 우리 프로젝트에서 AudioBridge를 사용할 때 장점은 무엇인가?  
3. **UI 설계**  
   - 사용자 입장에서 “방을 찾고 입장”하기 위해 필요한 화면 요소를 2가지 이상 나열하시오.

### **코멘트/팁**
- **Tip**: 비디오가 굳이 필요 없고, 다자 음성만 필요하다면 AudioBridge가 설정이 더 단순합니다(플러그인 구조도 간단).  
- **Tip**: Day 1은 다자 연결의 ‘개념 설계’와 ‘UI 구상’에 집중해, 기술적으로 어떻게 방을 만들어 관리할지 토론해보세요.  
- **Tip**: 실제 구현 시, Janus 측에서는 `create` 방식으로 Room을 명시 생성하거나, “auto room creation” 옵션을 켜 놓고 첫 Publish 때 자동 생성도 가능합니다.

---

## **Day 2**

### **목표**
- 안드로이드에서 **Room 생성/참가 시그널링** 전 과정을 코드로 작성  
- Janus에서 멀티 유저 입출력(오디오) 시 필요한 **Publisher/Subscriber** 로직 적용  
- 간단 “참여자 리스트” 표시 기능 도입

### **이론**
1. **멀티 유저 시그널링**  
   - 기존 1:1 Offer/Answer → 다자 구조에서 각 참가자가 **Publisher** 또는 **Subscriber**(혹은 양방향)  
   - Janus “join” 메시지, “publish” 메시지, “subscribe” 메시지에 대한 JSON 포맷  
2. **참여자 관리**  
   - 새 참가자가 들어오면 Room에 broadcast(“join event”)  
   - 레벨 측정(AudioLevels) or 안드로이드 UI(“XXX joined/left”) 구현 가능

### **실습**
1. **Room 생성 → Publish**  
   - 안드로이드: “Create Room” 버튼 클릭 시 Janus에 “create room” 요청 (혹은 특정 roomId로 `join`)  
   - “Publish” 단계에서 오디오 트랙을 SFU에 올림  
2. **Subscriber 연결**  
   - 다른 참가자(2번째 안드로이드 기기)에서 “Join Room” → `join` 메시지 → 서버 응답으로 “publisher list” 수신 → “subscribe” 요청 → 오디오 스트림 수신  
3. **UI 참여자 목록 업데이트**  
   - 방에 들어온 사람이 추가될 때 → ListView/RecyclerView 등 갱신(간단 구현 가능)  
   - (선택) 참가자별 오디오 레벨 표시(advanced)

### **숙제(퀴즈)**
1. **Publish vs Subscribe**  
   - 다자 연결에서 자신이 발언할 때 필요한 메시지(“publish”)와, 다른 이의 음성을 듣기 위해 필요한 메시지(“subscribe”)의 차이점 간단히 요약  
2. **시그널링 JSON**  
   - Janus `VideoRoom plugin`에서 “join” 메시지를 보낼 때 필요한 필드(예: `request`, `room`, `ptype`, `display`)를 2~3개 적어보시오.  
3. **UI 개선**  
   - 참여자 목록 표시 시, 기본적인 정보(닉네임/상태) 외에 어떤 것들을 표시하면 편리할지 생각해보기

### **코멘트/팁**
- **Tip**: 다자 연결은 “여러 명이 publish”할 수도 있지만, 가이드/보조 가이드만 publish하고 나머지는 subscribe만 할 수도 있습니다.  
- **Tip**: Janus `VideoRoom` plugin API Docs를 보면 join/publish/subscribe 각각의 JSON 예시가 잘 나와 있으니 참조하세요.

---

## **Day 3**

### **목표**
- **다자 동시 음성 통신** 테스트(최소 3명 이상)  
- 오디오 품질(비트레이트, 패킷화 간격) 기본 튜닝, 서버 리소스 확인  
- 방 생성/참여/나가기 전체 플로우에서 발생할 수 있는 오류 처리

### **이론**
1. **오디오 코덱/비트레이트 옵션**  
   - Opus 코덵: typical 20kbps~40kbps / 샘플링 레이트 48kHz  
   - SFU 서버 입장: 각 참가자 스트림을 Selective Forwarding → 서브스크라이버 수 × 비트레이트의 총량  
2. **서버 리소스**  
   - CPU: 암/복호화(SRTP) 부하, 동시 접속자 늘어날수록 상승  
   - 대역폭(Bandwidth): 업/다운 합산, Windows 서버 NIC 제한  
3. **오류 처리 시나리오**  
   - 방에 참여 중 서버가 다운되면? → Reconnect 로직  
   - 참여자가 방을 떠났는데 UI가 그대로 남아있으면? → 이벤트로 제거

### **실습**
1. **동시 접속 테스트(3~5명)**  
   - 에뮬레이터 + 실제 폰 2~3대 → 동일 Room ID로 join → 서로 음성 확인  
   - 서버 콘솔(Janus)에서 참가자 수, 각 유저 Publish/Subscribe 상태 모니터링  
2. **오디오 품질 설정**  
   - 안드로이드 WebRTC `AudioConstraints`(Bitrate / EchoCancellation / NoiseSuppression) 체크  
   - Janus Plugin 설정: maxopusbitrate 등 확인  
3. **예외/오류 처리**  
   - 방이 존재하지 않을 때 → 에러 메시지  
   - ICE 연결 실패 시 재시도, AudioTrack mute 시도, etc.

### **숙제(퀴즈)**
1. **오디오 비트레이트**  
   - Opus 코덱에서 20kbps와 64kbps의 차이는 어떤 상황에서 크게 체감될까?  
2. **서버 리소스**  
   - 동시 10명이 모두 발화(publish)할 경우, 서버가 처리해야 하는 암/복호화 작업량이 증가하는 이유를 간단히 서술하시오.  
3. **오류 처리**  
   - “Invalid room” 에러가 오면, 안드로이드 앱에서 어떤 UI를 띄워주어야 사용자 혼란이 줄어들까?

### **코멘트/팁**
- **Tip**: 안드로이드 에뮬레이터 여러 개를 동시에 구동하면 PC 성능이 많이 소모되므로, 다른 동료/친구의 실제 폰 여러 대로 테스트하면 편합니다.  
- **Tip**: 오디오 품질을 지나치게 높이면 4G/LTE 환경에서 끊길 가능성이 있습니다. 투어 시나리오라면 낮은 비트레이트+에코캔슬링+노이즈 제거가 더 중요한 경우가 많습니다.

---

## **Day 4**

### **목표**
- 가이드(Host)와 일반 참가자(Listener) 역할 분리:  
  - Host(가이드)만 publish, Listener는 subscribe만 하는 모드  
- UI/UX 개선: 방 목록, 초대 링크, 참여자 관리(Disconnect, Kick 등) 기초 도입  
- 품질 로그/통계 수집 기능(선택)

### **이론**
1. **가이드 & 청취자 구분**  
   - Janus Room Plugin: join 시 ptype=publisher or subscriber 선택 가능  
   - 앱 로직: Host UI(마이크 On/Off, 강제 음소거) vs Listener UI(오디오 수신 전용)  
2. **방 목록 & 초대**  
   - 로컬이나 서버 DB에 방 정보를 저장해둘지, 자동 생성 Room만 사용할지  
   - 초대 링크: “roomId=xxx” 형태

### **실습**
1. **Host 전용 모드**  
   - 앱에서 “가이드로 입장” 옵션 선택 시 `publish` 수행  
   - “청취자로 입장” 옵션 시 subscribe only  
2. **UI 요소**  
   - 방 목록(Activity or Fragment)에서 방 ID 선택 → 입장  
   - 참가자 리스트에 “Kick” 혹은 “Mute” 기능 추가(선택)  
3. **로깅/통계(선택)**  
   - Janus Admin API 혹은 plugin event를 받아, “현재 연결된 인원 수”를 앱에서 표시  
   - 안드로이드 Logcat에 Ping RTT, Packet Loss 정보 찍어보기

### **숙제(퀴즈)**
1. **Host & Listener**  
   - 가이드(Host)만 publish하고, 나머지는 subscribe만 하는 구조의 이점은 무엇인가?  
2. **방 목록**  
   - 앱에서 방 목록을 보여주려면, 서버 측에서 어떤 정보를 가져와야 할지 2가지 이상 서술  
3. **Mute/Kick 기능**  
   - 만약 가이드가 보조 가이드에게 발언 권한을 주거나 뺏으려면 어떤 접근 방법이 있을지 간단히 아이디어 제시

### **코멘트/팁**
- **Tip**: “Kick” 등의 강제 퇴장 기능은 실제 프로덕션 수준의 구현 시에는 인증/권한 관리가 필요합니다(간단 예제에서는 Skip 가능).  
- **Tip**: 안드로이드 UI가 복잡해질 수 있으므로, 우선 “TextView + Button” 정도로 구현한 뒤, 추후 디자인 개선을 고려하세요.

---

## **Day 5**

### **목표**
- Week 3 전체 마무리: **멀티 유저(방) 음성 통신** 프로토타입 완성  
- 3~5명 동시 연결 데모, 가이드/참가자 역할 실습  
- Git 버전 태깅(`v0.2-room-multi`) & 발표(시연)

### **이론**
1. **Week 3 마무리 개념**  
   - 다자 연결 시 발생하는 이벤트 흐름: Host `publish` → Listener `subscribe` → Janus `joined`/`leaving` notify  
   - 오디오 품질, 네트워크 환경(이동 중 Wi-Fi -> LTE 전환) 시 접속 유지 방안
2. **다음 단계(Week 4) 미리보기**  
   - 오류 처리/재연결 로직 고도화  
   - 음소거, 발언 권한 관리, 임베디드 기기(Raspberry Pi) 이식 등

### **실습**
1. **최종 다자 연결 데모**  
   - 방 만들기(가이드) → 나머지 2~3명(청취자) 입장 → 가이드 음성 방송 확인  
   - 2명 이상이 동시에 publish 가능하게 설정해보고, 오디오 혼합 상태 테스트  
2. **버그 수정 & 안정화**  
   - 동시에 여러 명이 들어오거나 나갈 때 UI가 꼬이지 않는지 확인  
   - Janus 로그에서 “No publisher in this room” 같은 에러 발생 여부 체크  
3. **Git 태깅 & 발표**  
   - `git tag -a v0.2-room-multi -m "Multi user room audio prototype"`  
   - 최종 발표(시연): 가이드가 안내 멘트 → 청취자들이 같은 방에서 듣는 장면 시연

### **숙제(퀴즈)**
1. **오디오 혼잡**  
   - 여러 발화자가 동시에 말하면 소리가 섞이는데, 어떤 솔루션(예: MCU, Active Speaker Detection)으로 개선할 수 있을지 간단히 생각해보기  
2. **Git**  
   - `git diff v0.1-audiotest v0.2-room-multi` 명령어가 어떤 기능을 하는지 설명  
3. **Week 4 대비**  
   - 다음 주에 Raspberry Pi 등 소형 컴퓨터로 배포를 시도한다고 했을 때, 미리 준비해야 할 사항 1가지를 써보기

### **코멘트/팁**
- **Tip**: Day 5에는 팀/동료와 실제 시연을 해보는 것이 중요합니다. 서로 다른 기기(각자 스마트폰)로 테스트해야 실전 감을 익힐 수 있습니다.  
- **Tip**: 이제 “가이드 1~2명 / 청취자 여러 명” 구조가 동작한다면, 투어 안내 시나리오의 기본 틀이 갖춰진 상태입니다.

---

# 요약

- **Week 3**는 **1:1**에서 확장하여, **방(Room) 기반 다자 연결**을 구현하고, **오디오 품질** 세팅과 기본 **오류 처리**를 다룹니다.  
- Day 1~2에서 **Room 개념**과 **시그널링**을 확장하고, Day 3~4에서 **다수 인원 동시 연결**, **오디오 튜닝**, **가이드/청취자 모드** 구현을 진행합니다.  
- Day 5에 **최종 다자 통신 데모**를 마무리하고, `v0.2-room-multi` 태그로 릴리스합니다.  
- 이로써 “가이드가 방을 만들고, 여러 사용자가 방에 들어와 동시에 음성 통신”이 가능해지며, 실질적인 **투어 가이드 시스템**의 초석이 마련됩니다.

이상으로 **Week 3 Day 1~5**의 상세 강의안이 완성되었습니다. 이후 주차(Week 4)에서는 더욱 심화된 안정화, 소형 컴퓨터 이식, STUN/TURN 최적화, 권한 관리 등을 다룰 수 있습니다. 필요하시면 이어서 요청해 주세요!