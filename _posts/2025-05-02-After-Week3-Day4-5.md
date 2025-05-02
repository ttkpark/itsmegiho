# W3 D4 : 역할 분리
- 역할 분리
- 방 목록 UI 기초
- 통계 로그 수집

_2025년 4월 20일, Kotlin + Jetpack Compose 기반 SFU 시스템 개발기 - Week3 Day4 회고_

---

## 🎯 Day 4 목표

- 가이드(Host) ↔ 일반 참가자(Listener) 역할 분리
- 가이드는 송신(Publish), 참가자는 수신(Subscribe)만
- 체크박스를 통한 음소거/해제 구현
- 참가자 리스트 동기화 (`muted`, `display` 상태 반영)
- Janus와 WebRTC 상태의 완전 동기화

---

## 🛠️ 1. 역할 분기 로직 구현

초기엔 `RoomEnterScreen`에서 `isGuide` 체크박스를 두고, RoomScreen으로 전달하여 역할을 구분했다.  
**AudioBridgeHandler.kt**에서 `configureRole()` 함수가 역할에 따라 `muted: true/false`를 Janus에 전송하도록 처리했다.

### 문제 1: "음소거 상태에서 접속하면 Janus 연결은 되지만 아무 소리도 안 들린다!"

**원인:** 음소거일 때 `initWebRTC()`를 호출하지 않아, WebRTC 자체 연결이 아예 안 되었던 것.  
**해결:** 수신 전용이라도 `prepareReceiverOnly()` → `onRemoteAnswer(jsep)`은 **항상** 호출되도록 분기 로직 재구성.

---

## 🎙️ 2. 음소거 해제 후 마이크가 작동하지 않음

처음에는 음소거 상태로 들어왔다가, 체크박스를 클릭해 음소거를 해제했을 때 **마이크가 다시 작동하지 않았다.**

### 시행착오:

- `removeTrack()` 하고 `addTrack()` 해도 작동 안 됨
- `peerConnection.addTrack()` 후 `createOffer()`로 재협상 시도 → `receiving: false`
- Janus 로그 상에서도 `"ICE failed"` 혹은 `"inactive"` SDP 트랙 포함됨

### 해결:

👉 결국 `AudioTrack` 객체를 새로 만들지 않고 **기존 트랙을 유지한 채**  
`audioTrack.setEnabled(true/false)`만 조절하는 방법으로 단순화.

> 💡 `setEnabled()`만으로도 WebRTC는 오디오를 무시하거나 활성화함.  
> 이 방식은 재협상도 필요 없고, 마이크 리소스도 재할당되지 않아 훨씬 안정적.

```kotlin
fun setMuteState(muted: Boolean) {
    audioTrack?.setEnabled(!muted)

    val tx = audioBridgeClient.generateTransactionId()
    val message = JSONObject().apply {
        put("janus", "message")
        put("session_id", sessionId)
        put("handle_id", handleId)
        put("transaction", tx)
        put("body", JSONObject().apply {
            put("request", "configure")
            put("muted", muted)
        })
    }
    audioBridgeClient.send(message)
}
````

---

## ✅ 3. UI ↔ 상태 연동

`RoomScreen.kt`에서 체크박스를 사용해 실시간으로 음소거 상태를 조절하도록 구현했다.

```kotlin
Checkbox(
    checked = isMuted.value.not(),
    onCheckedChange = { checked ->
        isMuted.value = !isMuted.value
        bridgeHandler.setMuteState(isMuted.value)
    }
)
```

> `isMuted`는 `remember { mutableStateOf(false) }` 로 상태 유지하고 UI와 즉시 연결됨

---

## 👥 4. 참가자 리스트 실시간 반영

처음에는 참가자 `ListView`가 갱신되지 않아 고민했다.
Janus에서 참여자 이벤트가 왔을 때 **중복 없이 리스트를 갱신**하려면, `id` 기준으로 비교하고 교체해야 했다.

```kotlin
onParticipantsUpdate = {
    for (map in it) {
        val idx = participantList.indexOfFirst { p -> p["id"] == map["id"] }
        if (idx != -1) participantList.removeAt(idx)
        participantList.add(map)
    }
}
```

또한 퇴장 시에는 `id`로 직접 비교해서 제거:

```kotlin
onParticipantsLeave = {
    val idx = participantList.indexOfFirst { p -> p["id"] == it }
    if (idx != -1) participantList.removeAt(idx)
}
```

---

## 💬 5. 출력 음량이 작다? → WebRTC의 AGC 설정 고민

송신자가 말하는 음성이 너무 작게 들려서:

* `googAutoGainControl` 해제 여부 확인
* `AudioTrack`에 대한 수동 gain 조절 불가
* `setEnabled(true)` 외 별도 마이크 게인 확대 불가 (단말 의존)

결론적으로는 수신 측에서 시스템 볼륨을 키우거나, 송신 측에서 **AGC 켜고 크게 말하기**가 현실적인 대안.

---

## ✨ 결과 정리

| 항목        | 내용                             |
| --------- | ------------------------------ |
| ✅ 역할 분기   | 가이드 ↔ 참가자 분리 완료                |
| ✅ 음소거 전환  | `setEnabled()` 기반 간단한 토글로 안정화  |
| ✅ UI 연결   | Compose의 `mutableState`로 즉시 반영 |
| ✅ 참여자 리스트 | `id` 기준 갱신 + 퇴장 반영 처리          |
| ❗ 소리 작음   | AGC 끄면 실제 작아짐 → 외부 조건 보완 필요    |

---

## 📌 다음 목표 (Day 5 예고)

* 전체 구조 정리 및 Git 태깅 (`v0.2-room-multi`)
* 가이드 ↔ 참가자 시나리오 데모 완성
* 발표 자료 정리 및 코드 문서화

---

> 💡 이 날 하루 동안만 약 4\~5개의 중대한 구조 개선을 해냈고,
> 실시간 음소거 토글은 상용 서비스 수준으로 안정화되었다.
> WebRTC는 복잡하지만, 명확하게 로직을 정리하면 이렇게 확실하게 동작한다!
