#!/bin/bash
#####################################
# 语音识别测试单例
# WorkMode任务脚本
# roubo_dai@san412.in
# San412.
#####################################

# 校园网认证客户端 命令行版 认证不成功

# 启动QQ
qq2012 > /dev/null 2>&1 &

# 打开一些默认网站
chromium-browser ym.163.com https://github.com/roubo

# 打开一些延续性的网站
web=$(cat /home/roubo/文档/webaddr)
if test -z "$web"
then
	echo "no webaddr."
else
	chromium-browser $web
fi


# 翻墙
expect ~/openPy/rouboSyS/Feel/Ears/TestLi/FOS/WorkmodeSets/SSH-D

# 打开火狐浏览器 和youtobe.com
firefox www.youtube.com & 

# 打开rouboSyS工作平台
gnome-terminal 

# 更新代码
gnome-terminal -e ~/openPy/rouboSyS/Feel/Ears/TestLi/FOS/WorkmodeSets/GIT.sh
