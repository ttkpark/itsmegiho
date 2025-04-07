---
layout: post
title: "After-Week1-Day3"
date : 2025-04-07
categories: [SFU_project]
---
# Janus Deploy 성공
- 아래는 설치 과정이다.
## ready for the Raspberry Linux

## Dependencies
- libnice, libsrtp, openssl, ...

## Build & Run
```bash
sudo apt update
sudo apt upgrade

sudo apt install libmicrohttpd-dev libjansson-dev \
  libssl-dev libsofia-sip-ua-dev libglib2.0-dev libopus-dev \
  libogg-dev libcurl4-openssl-dev liblua5.3-dev libconfig-dev \
  pkg-config gengetopt libtool automake autoconf \
  libnice-dev libsrtp2-dev libwebsockets-dev cmake

git clone https://github.com/meetecho/janus-gateway.git
cd janus-gateway
#git checkout v1.0.0  # 예시로 특정 버전 체크아웃 (또는 main) - 딱히 필요하진 않았았다.

sh autogen.sh
./configure --prefix=/opt/anus
make
sudo make install
sudo make configs

cd /opt/janus/bin
./janus
```

## Running Janus
- 웬일인지 설치 할 때 `/opt/janus` 폴더는 없고 `/opt/anus`는 있어서 `anus`폴더에서 진행했다.
- Terminal : janus 실행 터미널
```bash
cd /opt/anus/bin
./janus
```
- Terminal : HTTP Server 실행용 웹 앱
```bash
sudo apt install php
cd /opt/anus/share/janus/html 
# in dir /opt/anus/share/janus/html
php -S 0.0.0.0:8087
#[Mon Apr  7 12:44:21 2025] PHP 8.2.28 Development Server (http://0.0.0.0:8087) started
```

## Demo Tests
- EchoTest, VideoRoom plugin
- 테스트 성공.

## Janus 빌드 퀴즈
1. 빌드시 필요한 주요 라이브러리(libnice, libsrtp, openssl 등)가 각각 어떤 역할을 하는지 간단히 작성

| Name | roles |
| ----- | ----- |
| libmicrohttpd-dev      | 다른 프로젝트의 한 부분에서 HTTP 서버를 쉽게 돌리기 위한 툴 |
| libjansson-dev         | JSON 데이터를 바꾸고, encode 및 decode를 하게 해주는 라이브러리 |
| libssl-dev             | OpenSSL로, TLS(SSL), DTLS QUIC프로토콜을 지원하게 하는 라이브러리. |
| libsofia-sip-ua-dev    | User-agent SIP library. trasnport and ip layer 사이에서 VoIP 등을 음성/비디오 통신을 하는 데 사용. |
| libglib2.0-dev         | 형변환 라이브러리? |
| libopus-dev            |   |
| libogg-dev             |   |
| libcurl4-openssl-dev   |   |
| liblua5.3-dev          |   |
| libconfig-dev          |   |
| pkg-config             |   |
| gengetopt              |   |
| libtool                |   |
| automake               |   |
| autoconf               |   |
| libnice-dev            |   |
| libsrtp2-dev           |   |
| libwebsockets-dev      |   |
| cmake                  |   |
 
2. 데모 플러그인
- EchoTest와 VideoRoom의 차이를 1~2줄로 요약
- EchoTest는 VideoRoom과는 다른 툴로, EchoTest는 스스로 서버와 연결을 받아봄으로써 서버와 연결 및 기기의 전송을 테스트하는 툴이다. 반면 VideoRoom은 비디오 포함 방을 만들고, 그 방에서 초대자를 받아 대화할 수 있는 툴이다.

3. 오류 대처
- ./janus 실행 시 “Couldn’t open config file janus.cfg” 에러가 뜬다면 어떻게 해결할 수 있을까?
- 설정 폴더`opt/anus/etc/janus/` 속 janus.cfg의 존재 여부를 파악하고, 없을 때 동일한 이름 다른 확장자의 백업 파일을 복사하고 이름을 변경한다. 또는 sudo chmod `이름` 777로 권한을 수정한다.

4. 팁 내용인 `ststemctl`을 통해 매번 스스로 시작하는 서비스로 만들기
- [HowTo](https://medium.com/@benmorel/creating-a-linux-service-with-systemd-611b5c8b91d6)
- create file `/etc/systemd/system/janusservice.service`
```text
[Unit]
Description=janus demo service
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
RestartSec=1
User=monitor
ExecStart=./opt/anus/bin/janus

[Install]
WantedBy=multi-user.target
```
- create file `/etc/systemd/system/janusweb.service`
```text
[Unit]
Description=janus demo web service
After=network.target
StartLimitIntervalSec=0
[Service]
Type=simple
Restart=always
RestartSec=1
User=monitor
ExecStart=/opt/anus/share/janus/html php -S 0.0.0.0:8087

[Install]
WantedBy=multi-user.target
```
- `systemctl start janusservice` 실행
- `systemctl enable janusservice` 실행
- `systemctl status janusservice` 로 실행내용 파악

- `systemctl start janusweb` 실행
- `systemctl enable janusweb` 실행
- `systemctl status janusweb` 로 실행내용 파악

- 성공! 이젠 껐다 켜면 알아서 켜진다.