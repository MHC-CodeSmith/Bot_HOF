#!/usr/bin/env bash
# Script helper para inicializar o repositório git e fazer o primeiro commit
# Uso: ./scripts/init_git.sh [url-do-repo-github]

set -e

if [ -d .git ]; then
    echo "[!] Repositório git já inicializado."
    exit 1
fi

echo "[*] Inicializando repositório git..."
git init

echo "[*] Adicionando arquivos..."
git add .

echo "[*] Fazendo primeiro commit..."
git commit -m "TB4 sim: Dockerfile, run script, mapas e README"

echo "[*] Renomeando branch para main..."
git branch -M main

if [ -n "${1:-}" ]; then
    echo "[*] Adicionando remote origin: $1"
    git remote add origin "$1"
    echo "[*] Para fazer push, execute: git push -u origin main"
else
    echo "[*] Para adicionar o remote, execute:"
    echo "    git remote add origin git@github.com:<usuario>/<repo>.git"
    echo "    git push -u origin main"
fi

echo "[✓] Repositório inicializado com sucesso!"

