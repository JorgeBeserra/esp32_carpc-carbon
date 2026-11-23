#!/usr/bin/env bash
# Lê a versão do firmware em src/config.h e imprime linhas CHAVE=valor,
# prontas para "eval":
#   FW_BUILD    número do build (CFG_BUILD_NUM) - é ele que identifica a versão
#   FW_VERSION  texto da versão (CFG_VERSION)
#   FW_TAG      tag do release, build-<FW_BUILD>
set -euo pipefail

config="${1:-src/config.h}"

build=$(sed -n 's/^#define[[:space:]]\+CFG_BUILD_NUM[[:space:]]\+\([0-9]\+\).*/\1/p' "$config" | head -n1)
version=$(sed -n 's/^#define[[:space:]]\+CFG_VERSION[[:space:]]\+"\(.*\)".*/\1/p' "$config" | head -n1)

if [ -z "$build" ]; then
    echo "CFG_BUILD_NUM não encontrado em $config" >&2
    exit 1
fi

# %q escapa os espaços e aspas, então a saída também funciona com "eval"
printf 'FW_BUILD=%q\n' "$build"
printf 'FW_VERSION=%q\n' "${version:-$build}"
printf 'FW_TAG=%q\n' "build-$build"
