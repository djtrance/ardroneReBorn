# Protocolos de Comunicación del AR.Drone 2.0

## Arquitectura de Red

El AR.Drone 2.0 opera como un **Access Point WiFi**. El host (ground station) se conecta a la red del dron y se comunica mediante:

```
Host ──WiFi──┐
              │
      ┌───────┴───────┐
      │ AR.Drone 2.0  │
      │ 192.168.1.1   │
      └───────┬───────┘
              │
      ┌───────┴───────┐
      │ Cámaras       │
      │ IMU/Sensores  │
      │ DSP (H.264)   │
      └───────────────┘
```

## Puertos y Protocolos

### Control (Host → Dron) — AT Command Protocol

| Puerto | Protocolo | Dirección | Descripción |
|---|---|---|---|
| 5556 | UDP | Host → Dron | Comandos AT (takeoff, land, move, config) |

Los comandos AT tienen formato: `AT*<COMANDO>=<seq>[,<param1>,<param2>...]\r`

Ejemplos:
- `AT*REF=1,290718208\r` — Despegar
- `AT*PCMD=2,1,0,0,0,0\r` — Volar hacia adelante
- `AT*CONFIG=3,"general:navdata_demo","TRUE"\r` — Configurar

### NavData (Dron → Host) — Datos de Vuelo

| Puerto | Protocolo | Dirección | Descripción |
|---|---|---|---|
| 5554 | UDP | Dron → Host | Telemetría (altura, velocidad, batería, orientación) |

### Video Streaming (Dron → Host)

| Puerto | Protocolo | Dirección | Descripción |
|---|---|---|---|
| 5000 | UDP (RTP/AVP) | Dron → Host | Video H.264 + AAC en MP4 container |

SDP de ejemplo:
```
m=video 5000 RTP/AVP 96
a=rtpmap:96 H264/90000
a=framesize:96 1280-720
```

## Acceso al Sistema

### Telnet (shell remota)

```bash
telnet 192.168.1.1
# Login automático como root
```

Comandos útiles:
- `cat /firmware/version.txt` — versión del firmware
- `ps` — procesos activos
- `killall program.elf` — matar proceso nativo
- `cat /data/config.ini` — configuración actual

### FTP (transferencia de archivos)

```bash
ftp 192.168.1.1
# Login: anonymous / (any password)
```

## Configuración de Red

El dron se autoconfigura con IP estática. Se puede modificar:

```bash
# En el dron vía telnet:
sed -i 's/static_ip_address_base = 192.168.1./static_ip_address_base = 10.0.0./' /data/config.ini
sed -i 's/static_ip_address_probe = 1/static_ip_address_probe = 2/' /data/config.ini
```

## Comunicación Onboard

Para aplicaciones onboard, el código en C se comunica con:
- Cámaras: `/dev/video0` (frontal), `/dev/video1` (inferior) vía V4L2
- DSP: Codec Engine API (DSPLink)
- Control: puertos UDP locales para simular comandos

## Referencias

- AR.Drone 2.0 Developer Guide (SDK)
- https://github.com/flixr/paparazzi (implementación AT Command + NavData)
- `docs/video_formats.md` para detalles del stream de video
