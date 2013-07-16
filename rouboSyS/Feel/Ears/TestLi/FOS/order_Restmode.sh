#!/bin/bash
#####################################
# 语音识别测试单例
# RestMode任务脚本
# roubo_dai@san412.in
# San412.
#####################################

# 校园网认证客户端 命令行版 认证不成功

# 启动QQ
qq2012 > /dev/null  &

# 打开一些网站
chromium-browser www.hao123.com pl.yinyuetai.com

# 翻墙
expect ~/WorkSpace/openPy/rouboSyS/Feel/Ears/TestLi/FOS/WorkmodeSets/SSH-D

# 打开火狐浏览器 和youtobe.com
firefox www.youtube.com &

#conky
conky -c ~/.conkycolors/conkyrc &
