#!/usr/bin/env bash
# ==========================================================
# 一键提交并推送代码到 GitHub (PC / 开发设备端使用)
# ==========================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PKG_DIR}"

# 获取当前所在分支
BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo "=================================================="
echo ">> [推送] 当前分支: ${BRANCH}"
echo "=================================================="

# 1. 暂存所有改动
git add -A

# 2. 提交（支持自定义日志：./push_code.sh "提交信息"，默认自动带时间）
MSG="${1:-update code $(date '+%Y-%m-%d %H:%M:%S')}"
git commit -m "${MSG}" || echo ">> 提示: 没有检测到新的改动需要 commit"

# 3. 先变基拉取防冲突，再推送到远端
echo ">> [同步] 拉取远端更新 (rebase)..."
git pull --rebase origin "${BRANCH}"

echo ">> [推送] 正在推送到 GitHub..."
git push origin "${BRANCH}"

echo "=================================================="
echo ">> [完成] 代码已成功推送到 origin/${BRANCH}！"
echo "=================================================="
