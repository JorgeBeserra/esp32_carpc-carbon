#!/usr/bin/env bash
# Reúne os binários gerados pelo PlatformIO em uma pasta de saída:
#
#   firmware.bin    aplicativo (gravar em 0x10000)
#   bootloader.bin  bootloader (0x1000)
#   partitions.bin  tabela de partições (0x8000)
#   boot_app0.bin   seletor de partição OTA (0xe000)
#   firmware-completo.bin  imagem única com tudo acima, para gravar em 0x0
#   SHA256SUMS      checksums dos arquivos acima
#
# Os quatro primeiros usam os mesmos nomes que a skin (tools/updater.sh) espera.
#
# Uso: package_firmware.sh <pasta-do-build> <pasta-de-saida> [pasta-dos-pacotes-do-pio]
set -euo pipefail

build_dir="${1:?pasta do build (ex.: .pio/build/esp32doit-devkit-v1)}"
out_dir="${2:?pasta de saída}"
pio_packages="${3:-$HOME/.platformio/packages}"

# Parâmetros de flash usados pelo build (placa esp32doit-devkit-v1)
flash_mode=dio
flash_freq=40m
flash_size=4MB

for f in firmware.bin bootloader.bin partitions.bin; do
    [ -s "$build_dir/$f" ] || { echo "Faltou $build_dir/$f - o build gerou os binários?" >&2; exit 1; }
done

boot_app0=$(find "$pio_packages" -path '*framework-arduinoespressif32*' -name boot_app0.bin 2>/dev/null | head -n1)
[ -n "$boot_app0" ] || { echo "boot_app0.bin não encontrado em $pio_packages" >&2; exit 1; }

mkdir -p "$out_dir"
cp "$build_dir/firmware.bin" "$build_dir/bootloader.bin" "$build_dir/partitions.bin" "$out_dir/"
cp "$boot_app0" "$out_dir/boot_app0.bin"

python3 -m esptool --chip esp32 merge_bin \
    -o "$out_dir/firmware-completo.bin" \
    --flash_mode "$flash_mode" --flash_freq "$flash_freq" --flash_size "$flash_size" \
    0x1000  "$out_dir/bootloader.bin" \
    0x8000  "$out_dir/partitions.bin" \
    0xe000  "$out_dir/boot_app0.bin" \
    0x10000 "$out_dir/firmware.bin"

(cd "$out_dir" && sha256sum firmware.bin bootloader.bin partitions.bin boot_app0.bin firmware-completo.bin > SHA256SUMS)
echo "Binários em $out_dir:"
ls -l "$out_dir"
