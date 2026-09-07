#!/bin/bash

# 创建 tmux 会话
tmux new-session -d -s my_session

# 垂直分屏
tmux split-window -h

# 右侧窗格：先启动 Python 服务器
tmux send-keys -t my_session:0.1 'conda deactivate && cd ~/web_s/shop1009/web && python server.py' C-m

# 左侧窗格：先进入目录，然后等待3秒再执行
tmux send-keys -t my_session:0.0 'cd ~/爱仕达标准货架/market_simple_2026_1_21_using/build && sleep 6 && echo dongguan | sudo -S ./move' C-m

# 附加到会话
tmux attach -t my_session