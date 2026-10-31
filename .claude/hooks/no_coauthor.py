#!/usr/bin/env python3
"""Не пропускать коммит со следами соавторства.

Правило репозитория: сообщение коммита — одна строка, без `Co-Authored-By`
и без подписей инструмента. Хук ловит это до того, как коммит уйдёт в
историю, где вычищать его пришлось бы перезаписью веток.

Один и тот же файл лежит в `.claude/hooks/` и `.codex/hooks/`: правите
один — правьте и второй.
"""

from __future__ import annotations

import json
import re
import sys

# `git commit` и `git -C <путь> commit`, но не `git log | grep commit`.
# shortcut: сообщение из файла (`git commit -F`) не читается; читать файл, если агенты начнут так коммитить
GIT_COMMIT = re.compile(r"\bgit\b[^|;&\n]*\bcommit\b")
FORBIDDEN = (
    re.compile(r"co-authored-by", re.IGNORECASE),
    re.compile(r"generated with .*(claude|codex)", re.IGNORECASE),
    re.compile(r"🤖"),
)


def main() -> int:
    try:
        payload = json.load(sys.stdin)
    except ValueError:
        return 0

    command = (payload.get("tool_input") or {}).get("command")
    if not isinstance(command, str) or not GIT_COMMIT.search(command):
        return 0

    if any(pattern.search(command) for pattern in FORBIDDEN):
        print(
            "Коммит отклонён: сообщение — одна строка, без Co-Authored-By "
            "и без подписей инструмента.",
            file=sys.stderr,
        )
        return 2

    return 0


if __name__ == "__main__":
    sys.exit(main())
