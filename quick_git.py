#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.system("git pull")
os.system("git add .")
print("\n"*5)
print("!!중요!!")
print("수정한 것이 없었다면 Ctrl+C만 누르세요.")
print("무엇을 수정하셨습니까? : ", end="")
commit_reason = input()
os.system("git commit -m \"%s\""%commit_reason)
os.system("git push")
print("quick_git.py가 종료되었습니다.")
