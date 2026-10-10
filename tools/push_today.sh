#!/bin/bash
# 下班把今天的改动提交并推到自己的 GitHub。
# 用法: tools/push_today.sh "今天改了什么"
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ $# -lt 1 ]]; then
  echo "用法: tools/push_today.sh \"今天改了什么\""
  exit 1
fi

name="$(git config --local --get user.name || true)"
email="$(git config --local --get user.email || true)"
echo "本次提交作者: ${name} <${email}>"
if [[ -z "${name}" || -z "${email}" ]]; then
  echo "本仓库还没有自己的作者，已停止。"
  exit 1
fi
if [[ "${name}" == "song" || "${name}" == "Coleram" ]]; then
  echo "作者仍是旧身份 ${name}，已停止。"
  exit 1
fi

git add -A -- . \
  ':(exclude)*.bak*' \
  ':(exclude)**/*.bak*' \
  ':(exclude)tmp_calib_session_swap/**' \
  ':(exclude).env' \
  ':(exclude)**/.env' \
  ':(exclude)*.pem' \
  ':(exclude)*.key' \
  ':(exclude)credentials.json'

if ! git diff --cached --quiet; then
  git commit -m "$1"
else
  echo "没有新的改动需要提交。"
fi

git push -u origin HEAD
echo "已推到 GitHub: SuperJack0228/T170-WheeledRobot-Factory-Auto"
