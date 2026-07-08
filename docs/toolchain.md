# Toolchain ARM para Cross-Compilation

## Paquetes Disponibles

### parrot-tools-linuxgnutools-2012.03 (i386)

- gcc 4.4.5
- Para AR.Drone 2.0 (firmware 2.x)
- Solo 32-bit (i386)

### parrot-tools-linuxgnutools-2016.02-linaro (amd64)

- gcc 4.6.x (Linaro)
- Compatible con AR.Drone 2.0
- 64-bit (amd64 host)

Fuente: https://github.com/tudelft/toolchains

## Instalación en Linux

```bash
wget https://github.com/tudelft/toolchains/raw/master/parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb
sudo dpkg -i parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb
sudo apt-get install -f  # resolver dependencias
```

Los binarios se instalan en `/opt/arm-none-linux-gnueabi/` o `/usr/bin/` con prefijo `arm-none-linux-gnueabi-`.

## Instalación en macOS

### Opción 1: Docker (recomendada)

```bash
docker pull ubuntu:22.04
docker run -it -v $(pwd):/workspace ubuntu:22.04 /bin/bash
# Dentro del contenedor:
apt update && apt install -y wget dpkg
wget https://github.com/tudelft/toolchains/raw/master/parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb
dpkg -i parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb
```

### Opción 2: Extraer toolchain manualmente

```bash
wget https://github.com/tudelft/toolchains/raw/master/parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb
mkdir -p /opt/parrot-toolchain
cd /opt/parrot-toolchain
ar x ~/Downloads/parrot-tools-linuxgnutools-*.deb
tar -xzf data.tar.gz
export PATH=/opt/parrot-toolchain/usr/bin:$PATH
```

## Verificar Instalación

```bash
arm-none-linux-gnueabi-gcc --version
arm-none-linux-gnueabi-gcc -v 2>&1 | grep Target
# Debe mostrar: Target: arm-none-linux-gnueabi
```

## Sysroot

Para compilar programas que se ejecutarán EN el dron, se necesita un sysroot con las librerías del dron:

```bash
# Extraer librerías del dron vía FTP
ftp 192.168.1.1
cd /lib
get libc.so.6
get libpthread.so.0
get libm.so.6
# ... todas las librerías necesarias
```

O alternativamente, copiar la toolchain sysroot incluida en los paquetes de visión (`arm_light.tgz`).

## Uso

```makefile
CC = arm-none-linux-gnueabi-gcc
AR = arm-none-linux-gnueabi-ar
CFLAGS = -Os -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=softfp
LDFLAGS = -static-libgcc -L$(SYSROOT)/lib
```

## Prefijos de Toolchain

| Comando | Propósito |
|---|---|
| `arm-none-linux-gnueabi-gcc` | Compilador C |
| `arm-none-linux-gnueabi-g++` | Compilador C++ |
| `arm-none-linux-gnueabi-ar` | Archiver (librerías estáticas) |
| `arm-none-linux-gnueabi-strip` | Eliminar símbolos (reducir tamaño) |
| `arm-none-linux-gnueabi-objdump` | Desensamblador |
| `arm-none-linux-gnueabi-readelf` | Info de ELF |

## Referencias

- https://github.com/tudelft/toolchains — .deb packages
- https://github.com/flixr/paparazzi-portability-support — build scripts
- `build/Makefile` y `build/Makefile.include` en este repositorio
