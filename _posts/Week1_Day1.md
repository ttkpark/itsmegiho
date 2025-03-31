# Week 1 Day 1 – 기초 다지기 (Git/GitHub & WebRTC 개요)

안녕하세요! **Week 1 Day 1** 수업 자료입니다.  
이 강의안을 차근차근 따라가면 **Git/GitHub 기초**를 익히고, **WebRTC 기본 개념**을 복습하며, **SFU(Mesh/MCU) 차이**를 간단히 파악할 수 있습니다.

---

## 1. 목표

1. **Git/GitHub** 기초 습득
   - 버전 관리를 위한 **로컬 저장소 초기화**, **원격 저장소 연결**, **기본 명령어**(`commit`, `push`, `pull`) 학습  
   - 간단한 협업 흐름(Branch, Merge) 개념 맛보기  

2. **WebRTC 기본 개념** 복습
   - P2P 통신에서 SDP, ICE Candidate 개념 재확인  
   - 시그널링(Signaling)이 왜 필요한지 이해  

3. **SFU / MCU / Mesh** 비교
   - 서버 기반의 WebRTC 다자 연결 모델(SFU)의 장단점  
   - Day 1에는 개념만, 실제 설치나 코드 작성은 이후 진행  

---

## 2. 이론

### 2.1 Git/GitHub 기초

1. **버전 관리의 필요성**  
   - 협업 시 파일 이력을 추적하고, 이전 버전으로 롤백할 수 있음  
   - 여러 명이 동시에 개발해도 충돌을 최소화

2. **Git의 기본 개념**  
   - **로컬 저장소**(Local Repository): 내 PC에 존재하는 `.git` 폴더  
   - **원격 저장소**(Remote Repository): GitHub, GitLab, Bitbucket 등 서버에 있는 리포지토리  
   - **기본 흐름**:  
     1) `git add` → 스테이징(Staging)  
     2) `git commit` → 로컬 저장소에 기록  
     3) `git push` → 원격 저장소에 반영

3. **GitHub 소개**  
   - 가장 인기 있는 호스팅 플랫폼 중 하나  
   - 저장소(Repo) 생성, 협업(이슈, Pull Request) 관리, Wiki, Actions(CI/CD) 등 기능  

### 2.2 WebRTC 기초 개념 복습

1. **P2P와 시그널링**  
   - WebRTC는 **SDP**(Session Description Protocol)를 교환해 오디오/비디오 코덱, 암호화 방식 등을 결정  
   - **ICE Candidate**로 NAT/방화벽 뒤에서도 P2P 연결을 시도  
   - **시그널링**: 누가 누구와 연결할 것인지 정보를 주고받는 과정. (Firebase, Socket.io, Custom Server 등 다양)

2. **SDP & ICE Candidate 흐름**  
   - **Offer**(A가 B에게 보냄) → **Answer**(B가 A에게) → 서로 ICE Candidate 전송  
   - 성공 시 P2P 미디어 채널 형성

3. **Mesh / MCU / SFU**  
   - **Mesh**: 모든 클라이언트가 서로 P2P 연결, N명일 때 연결 수가 많아짐  
   - **MCU**: 서버에서 모든 영상을 디코딩 후 믹싱, 다시 인코딩 (서버 부담 큼)  
   - **SFU**: 서버는 **Selective Forwarding**만 수행. 중간에서 미디어를 받아 그대로(or 일부 계층 선택) 전달 → 확장성과 성능이 좋음

---

## 3. 실습

아래 절차를 **순서대로** 따라 하시면 됩니다.  
오늘은 Git/GitHub를 설치하고 간단한 로컬 저장소를 만든 뒤, GitHub 원격 저장소와 연동하는 과정을 중심으로 진행합니다.  

### 3.1 Git 설치 & 환경 설정

1. **Git 설치 확인**  
   - 윈도우 환경이라면 [Git 공식 다운로드 페이지](https://git-scm.com/download/win)에서 다운로드 가능  
   - 이미 설치되어 있다면 버전 확인:
     ```bash
     git --version
     ```
   - 출력 예: `git version 2.39.1.windows.1`

2. **기본 설정**  
   - 사용자 정보 등록 (처음 한 번만):
     ```bash
     git config --global user.name "내이름"
     git config --global user.email "내이메일@example.com"
     ```
   - 설정 확인:
     ```bash
     git config --list
     ```
   - 정상적으로 입력되었는지 `user.name`, `user.email` 항목을 확인

3. **(인터넷 검색 활동)**  
   - Git 관련 궁금한 점이 생기면 “Git tutorial” 또는 “Git basic commands” 등으로 구글 검색을 해보세요.  
   - Git 공식 문서나 Atlassian Git Tutorial 등이 참고하기 좋습니다.

### 3.2 Git 초기화 & 첫 커밋

1. **프로젝트 폴더 생성**  
   - 예: `C:\Projects\SFUProject` 폴더를 만들고, `cmd` 또는 `Git Bash`에서 이동:
     ```bash
     cd C:\Projects\SFUProject
     ```
2. **Git 초기화**  
   - 현재 폴더에 `.git` 폴더가 생김:
     ```bash
     git init
     ```
   - 출력 예: `Initialized empty Git repository in C:/Projects/SFUProject/.git/`

3. **파일 생성 & 첫 커밋**  
   - 텍스트 에디터 또는 VSCode 등으로 `README.md` 파일을 만듭니다.  
   - 내용 예시:
     ```markdown
     # SFU Project
     This project will explore WebRTC SFU, starting with Git basics on Day 1.
     ```
   - Git으로 스테이징 후 커밋:
     ```bash
     git add .
     git commit -m "Initial commit: Added README.md"
     ```
   - 출력 예: `[main (root-commit) 1a2b3c4] Initial commit: Added README.md`

### 3.3 GitHub 원격 저장소 연결

1. **GitHub에서 새 Repo 생성**  
   - GitHub 로그인 → 우측 상단 New → Repository name 예) `sfu-project`  
   - Private/Public 여부 설정 후 **Create Repository**  
2. **원격 저장소 추가**  
   - 생성된 리포지토리 URL 확인 (예: `https://github.com/username/sfu-project.git`)  
   - 로컬에서 연결:
     ```bash
     git remote add origin https://github.com/username/sfu-project.git
     ```
   - 잘 연결되었는지 확인:
     ```bash
     git remote -v
     ```
   - 출력 예:  
     ```
     origin  https://github.com/username/sfu-project.git (fetch)
     origin  https://github.com/username/sfu-project.git (push)
     ```
3. **첫 푸시**  
   - 최신 코드를 원격에 업로드:
     ```bash
     git push -u origin main
     ```
   - 첫 푸시인 경우 `-u origin main`으로 브랜치를 추적

### 3.4 WebRTC 기본 구조 복습 (간단 P2P)

1. **폴더/파일 추가**  
   - `p2p-demo/` 폴더 생성  
   - 여기서 WebRTC 관련 실습(HTML/JS 등)을 진행할 수도 있고, 안드로이드 프로젝트를 만들 수도 있음  
   - 아직 구현은 하지 않고 구조만 잡아둠  
2. **P2P 개념 정리 문서 작성**  
   - `docs/` 폴더 안에 `webrtc-p2p-overview.md` 파일 생성  
   - 예시 내용:
     ```markdown
     # WebRTC P2P Overview
     - SDP Offer/Answer
     - ICE Candidate Gathering
     - STUN/TURN usage
     ```
   - 추가하고 커밋/푸시:
     ```bash
     git add .
     git commit -m "Add basic WebRTC P2P overview doc"
     git push
     ```
3. **(인터넷 검색 활동)**  
   - “WebRTC official samples”를 검색: <https://webrtc.github.io/samples/>  
   - 샘플 페이지(기본 카메라 테스트, Audio test 등) 둘러보고 개념 리뷰  

---

## 4. 숙제(퀴즈)

1. **Git 개념 퀴즈**
   - `git clone`과 `git pull`의 차이를 간단히 써보세요.  
   - `git add .` 명령어는 어떤 역할을 하나요?

2. **WebRTC 기본 퀴즈**
   - P2P 연결에서 SDP가 어떤 정보를 담는지 2가지 이상 쓰세요.  
   - ICE Candidate는 왜 필요한가요?

3. **SFU/MCU/Mesh 단답형**
   - SFU 구조가 Mesh 대비 가지는 장점 2가지는?

(숙제 답안을 **README.md**나 **docs/Day1-quiz.md** 등에 메모하고, 간단한 커밋 후에 `git push`로 제출해보세요!)

---

## 5. 코멘트/팁

1. **Git 작업 흐름 정착**  
   - Day 1에 배운 Git/GitHub 기초는 앞으로의 모든 작업(코드 작성, 문서화)에 활용됩니다.  
   - **소소한 수정 사항도 자주 커밋**하고, 작업 단위를 잘게 나누는 습관을 들이세요.

2. **WebRTC 혼동 주의**  
   - 아직은 P2P 개념 복습 수준이니, 상세 구현은 Week 2에서 안드로이드 프로젝트 + Janus SFU로 진행할 겁니다.  
   - P2P에서 쓰인 SDP/ICE 흐름이 SFU에서도 거의 동일하게 쓰입니다.

3. **오늘의 핵심**  
   - **Git**으로 “내가 뭘 했는지” 이력을 남기는 건 매우 중요합니다.  
   - **WebRTC 기초**(SDP, ICE)를 확실히 이해하면 이후 SFU 서버 연결 때 훨씬 수월합니다.

4. **다음 수업 예고**  
   - **Day 2**에는 SFU, MCU, Mesh의 차이를 좀 더 깊이 파고들며 오픈 소스 SFU(Janus 등)를 비교해볼 예정입니다.  
   - 본격적으로 서버 설치 전, 어떤 SFU를 쓸지 의사결정을 하게 됩니다.

---

# 마무리

오늘은 **Git/GitHub**를 설치하고, **로컬 저장소**와 **원격 저장소**를 연결하는 방법을 익혔습니다.  
또한 WebRTC **P2P 개념**을 살짝 복습하며, **SFU/MCU/ Mesh** 개념을 맛보기 정도로 확인했습니다.

> **오늘 하루의 최종 목표 달성 체크**  
> - [ ] Git 설치 후 `git --version`으로 버전 확인  
> - [ ] GitHub에 “sfu-project” 등 새 Repo 만들고 로컬에서 `git push` 성공  
> - [ ] WebRTC 기초 개념(Offer/Answer, ICE) 복습 문서 작성/커밋  
> - [ ] SFU/MCU/Mesh 각각 핵심 포인트 1문장씩 정리

하루 수업을 모두 마치고 숙제를 끝냈다면, **Week 1 Day 1** 계획을 **완료**하신 것입니다.  
다음 수업 때 뵙겠습니다!
